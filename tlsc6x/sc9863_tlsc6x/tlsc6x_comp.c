/*
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *
 */
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/kthread.h>
//#include <linux/wakelock.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/suspend.h>
#include <linux/irq.h>
#include "tlsc6x_main.h"
#include "tlsc6x_update.h"
#include "tlsc6x_gesture_binlib.h"

#define TLSC_HAVE_BUILD_CFG // close:undef
unsigned int g_tlsc6x_cfg_ver = 0;
unsigned short g_tlsc6x_chip_code = 0;
struct i2c_client *g_tlsc6x_client;


#ifdef TLSC_AUTO_UPGRADE
static int new_idx_active = -1;
unsigned short  buidIn_tls_tp_lut[][102]={
    {
        #include "tls_tp_zhenhai_e4020_v03.h"
    },
    {
        #include "tls_tp_yuye_e4020_v03.h"
    }
};
#endif

// Telink CTP

struct tp_cf_data{
    unsigned short data[102];
};
typedef struct __test_cmd_wr{
    //offset 0;
    unsigned char id; //cmd_id;
    unsigned char idv; //inverse of cmd_id
    unsigned short d0; //data 0
    unsigned short d1; //data 1
    unsigned short d2; //data 2
    //offset 8;
    unsigned char resv;  //offset 8
    unsigned char tag;   //offset 9
    unsigned short chk;  // 16 bit checksum
    unsigned short s2Pad0;  //
    unsigned short s2Pad1;  //
}ctp_test_wr_t;

typedef struct __test_cmd_rd{
    //offset 0;
    unsigned char id; //cmd_id;
    unsigned char cc; //complete code
    unsigned short d0; //data 0
    unsigned short sn; //session number
    unsigned short chk;  // 16 bit checksum
}ctp_test_rd_t;
#define DIRECTLY_MODE   (0x0)
#define DEDICATE_MODE   (0x1)
#define MAX_TRX_LEN (64)   // max IIC data length
#define CMD_ADDR    (0xb400)
#define RSP_ADDR    (0xb440)
#define MTK_TXRX_BUF    (0xcc00)  // 1k, buffer used for memory r &w

#define LEN_CMD_CHK_TX  (10)
#define LEN_CMD_PKG_TX  (16)

#define LEN_RSP_CHK_RX  (8)
#define MAX_BULK_SIZE    (1024)
unsigned short tl_target_cfg[102];
unsigned short tl_buf_tmpcfg[102];
unsigned char cmd_2dma_42bd[6]={/*0x42, 0xbd, */0x28, 0x35, 0xc1, 0x00, 0x35, 0xae};  // to direct memory access mode

// in directly memory access mode
// RETURN:0->pass else->fail
int tlsc6x_read_bytes_u16addr(struct i2c_client *client, u16 addr, u8 *rxbuf, u16 len)
{
    int err = 0;
    int retry = 0;
    u16 offset = 0;
    u8 buffer[2];

    struct i2c_msg msgs[2] ={
        {
            .addr = client->addr,
            .flags = 0,
            .len = 2,   // 16bit memory address
            .buf = buffer,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
        },
    };

    if(NULL == rxbuf){
        return -1;
    }

    //mutex_lock(&g_mutex_i2c_access);

    while(len > 0){
        buffer[0] = (u8)((addr+offset)>>8);
        buffer[1] = (u8)(addr+offset);

        msgs[1].buf = &rxbuf[offset];
        if(len > MAX_TRX_LEN){
            len -= MAX_TRX_LEN;
            msgs[1].len = MAX_TRX_LEN;
        }else{
            msgs[1].len = len;
            len = 0;
        }

        retry = 0;
        while(tlsc6x_i2c_Read(client, buffer, 2, &rxbuf[offset], msgs[1].len) < 0){
            if(retry++ == 3){
                err = -1;
                break;
            }
        }
        offset += MAX_TRX_LEN;
        if(err < 0){
            break;
        }
    }

    //mutex_unlock(&g_mutex_i2c_access);

    return err;
}

// in directly memory access mode
// RETURN:0->pass else->fail
int tlsc6x_write_bytes_u16addr(struct i2c_client *client, u16 addr,const u8 *txbuf, u16 len)
{
    u8 buffer[MAX_TRX_LEN];
    u16 offset = 0;
    u8 retry = 0;
    int err = 0;

    struct i2c_msg msg ={
        .addr = client->addr,
        .flags = 0,
        .buf = buffer,
    };

    if(NULL == txbuf){
        return -1;
    }

    //mutex_lock(&g_mutex_i2c_access);

    while(len){
        buffer[0] = (u8)((addr+offset)>>8);
        buffer[1] = (u8)(addr+offset);

        if(len > (MAX_TRX_LEN-2)){ // (sizeof(addr)+payload) <= MAX_TRX_LEN
            memcpy(&buffer[2], &txbuf[offset], (MAX_TRX_LEN-2));
            len -= (MAX_TRX_LEN-2);
            offset += (MAX_TRX_LEN-2);
            msg.len = MAX_TRX_LEN;
        }else{
            memcpy(&buffer[2], &txbuf[offset], len);
            msg.len = len + 2;
            len = 0;
        }

        retry = 0;
        while(tlsc6x_i2c_Write(client, buffer, msg.len) < 0){
            if(retry++ == 3){
                err = -1;
                break;
            }
        }
        if(err < 0){
            break;
        }
    }

    //mutex_unlock(&g_mutex_i2c_access);

    return err;
}


// <0 : i2c error
// 0: direct address mode
//// 1: protect mode
 int tlsc6x_get_i2cmode(void)
{
    u8 regData[4];

    if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0x01, regData, 3)){
        return -1;
    }
    if(((regData[0]>>1) == (u8)(g_tlsc6x_client->addr)) && (0x01 == regData[2])){
        tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0x28, regData, 2);
        if((0x10==regData[0]) && (0xdf==regData[1])){
            return DIRECTLY_MODE;
        }
    }

    return DEDICATE_MODE;
}

// 0:successful
int tlsc6x_set_dd_mode(void)
{    
    int mod = -1;
    int retry = 0;

    if(DIRECTLY_MODE == tlsc6x_get_i2cmode()){
        return 0;
    }
    
    while(retry++ < 5){
        tlsc6x_write_bytes_u16addr(g_tlsc6x_client, 0x42bd, cmd_2dma_42bd, 6);
        msleep(30);
        mod = tlsc6x_get_i2cmode();
        if(DIRECTLY_MODE == mod){
            break;
        }
    }

    if(DIRECTLY_MODE == mod){
        return 0;
    }else{
        return -1;
    }
}

// 0:successful
int tlsc6x_set_nor_mode(void)
{    
    int mod = -1;
    int retry = 0;
    u8 reg = 0x05;
    
    while(retry++ < 5){
        tlsc6x_write_bytes_u16addr(g_tlsc6x_client, 0x03, &reg, 1);
        msleep(5);
        mod = tlsc6x_get_i2cmode();
        if(DEDICATE_MODE == mod){
            break;
        }
        msleep(50);
    }
    if(DEDICATE_MODE == mod){
        return 0;
    }else{
        return -1;
    }
}

// ret=0 : successful
// write with read-back check, in dd mode
static int tlsc6x_bulk_down_check(const u8* pbuf, u16 addr, u16 len)
{
    unsigned int j, k, retry;
    u8 rback[128];

    while(len){
        k = (len<128)?len:128;
        retry = 0;
        do{
            rback[k-1] = pbuf[k-1]+1;
            tlsc6x_write_bytes_u16addr(g_tlsc6x_client, addr, pbuf, k);
            tlsc6x_read_bytes_u16addr(g_tlsc6x_client, addr, rback, k);
            for(j=0; j<k; j++){
                if(pbuf[j] != rback[j]){
                    break;
                }
            }
            if(j >= k){
                break;  // match
            }
        } while(++retry < 3);
        
        if(j < k){
            break;
        }
        
        addr += k;
        pbuf += k;
        len -= k;
    }
    
    return (int)len;
}
static u16 tlsc6x_checksum_u16(u16 * buf, u16 length)
{
    unsigned short sum, len, i;
    sum =0;
    
    len = length >>1;
    
    for(i=0; i<len; i++){
        sum += buf[i];
    }

    return sum;
}
static u32 tlsc6x_checksumEx(u8 * buf, u16 length)
{
    u32 combChk;
    u16 k, check, checkEx;
    
    check = 0;
    checkEx = 0;
    for(k=0; k<length; k++){
        check += buf[k];
        checkEx += (u16)(k*buf[k]);
    }
    combChk = (checkEx<<16)|check;

    return combChk;
    
}

// 0:successful
static int tlsc6x_download_ramcode(const u8* pcode, u16 len)
{
    u8 dwr, retry;
    int ret = -2;

    if(tlsc6x_set_dd_mode()){
        return -1;
    }

    dwr = 0x05;
    if(0 == tlsc6x_bulk_down_check(&dwr, 0x0602, 1)){   // stop mcu
        dwr = 0x00;
        tlsc6x_bulk_down_check(&dwr, 0x0643, 1);            //  disable irq
    }else{
        return -1;
    }
    if(0 == tlsc6x_bulk_down_check(pcode, 0x8000, len)){
        dwr = 0x88;
        retry = 0; 
        do{
            ret = tlsc6x_write_bytes_u16addr(g_tlsc6x_client, 0x0602, &dwr, 1);
        }while((++retry < 3) && (0 != ret));
    }

    //msleep(50);   // let caller decide the delay time
    
    return ret;
}

// return 0=successful: send cmd and get rsp.
static int tlsc6x_cmd_send(ctp_test_wr_t *ptchcw, ctp_test_rd_t *pcr)
{
    int ret;
    u32 retry;

    retry = 0;
    tlsc6x_write_bytes_u16addr(g_tlsc6x_client, RSP_ADDR, (u8*)&retry, 1);

    // send command
    ptchcw->idv = ~(ptchcw->id);
    ptchcw->tag = 0x35;
    ptchcw->chk = 1+ ~(tlsc6x_checksum_u16((u16*)ptchcw, LEN_CMD_CHK_TX));
    ptchcw->tag = 0x30;
    ret = tlsc6x_write_bytes_u16addr(g_tlsc6x_client, CMD_ADDR, (u8*)ptchcw, LEN_CMD_PKG_TX);
    if(ret){
        goto exit;
    }
    ptchcw->tag = 0x35;
    ret = tlsc6x_write_bytes_u16addr(g_tlsc6x_client, CMD_ADDR+9, (u8*)&(ptchcw->tag), 1);
    if(ret){
        goto exit;
    }

    // polling rsp, the caller must init rsp buffer. 
    ret = -1;
    retry = 0;
    while(retry++ < 100){    // 2s
        msleep(20);
        if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, RSP_ADDR, (u8*)pcr, 1)){
            break;
        }

        if(ptchcw->id != pcr->id){
            continue;
        }
        //msleep(50);
        tlsc6x_read_bytes_u16addr(g_tlsc6x_client, RSP_ADDR, (u8*)pcr, LEN_RSP_CHK_RX);
        if(!tlsc6x_checksum_u16((u16 *)pcr, LEN_RSP_CHK_RX)){
            if((ptchcw->id==pcr->id) && (0==pcr->cc)){
                ret = 0;
            }
        }    
        break;
    }
exit:
    // clean rsp buffer
    //retry = 0;
    //tlsc6x_write_bytes_u16addr(g_tlsc6x_client, RSP_ADDR, (u8*)&retry, 1);

    return ret;
        
}

// return 0=successful
static int tlsc6x_read_burn_space(u8 * pdes, u16 adr, u16 len)
{    
    int rsp;
    u32 left = len;
    u32 combChk, retry;
    ctp_test_wr_t m_cmd;
    ctp_test_rd_t m_rsp;    

    m_cmd.id  = 0x31;
    m_cmd.resv = 0x03;    
    while(left){
        len = (left>MAX_BULK_SIZE)?MAX_BULK_SIZE:left;
        
        m_cmd.d0 = adr;
        m_cmd.d1 = len;

        rsp = -1;
        retry = 0;
        while(retry++ < 3){
            m_rsp.id = 0;
            if(0x0 == tlsc6x_cmd_send(&m_cmd, &m_rsp)){
                tlsc6x_read_bytes_u16addr(g_tlsc6x_client, MTK_TXRX_BUF, pdes, len);
                combChk = tlsc6x_checksumEx(pdes, len);
                if(m_rsp.d0 == (unsigned short)combChk){
                    if(m_rsp.sn == (unsigned short)(combChk>>16)){
                        rsp = 0;
                        break;
                    }
                }
            }
        }
        
        if(rsp < 0){
            break;
        }
        left -= len;
        adr += len;
        pdes += len;
    }
    
    return rsp;
}

static int tlsc6x_write_burn_space(u8 * psrc, u16 adr, u16 len)
{  
    int rsp=0;
    u16 left = len;
    u32 retry, combChk;
    ctp_test_wr_t m_cmd;
    ctp_test_rd_t m_rsp;  

    if((adr + len) > 0x7f90){
        return -1;
    }
    m_cmd.id  = 0x30;
    m_cmd.resv = 0x11;
    
    while(left){
        len = (left>MAX_BULK_SIZE)?MAX_BULK_SIZE:left;
        combChk = tlsc6x_checksumEx(psrc, len);
        
        m_cmd.d0 = adr;
        m_cmd.d1 = len;
        m_cmd.d2 = (u16)combChk;
        m_cmd.s2Pad0 = (u16)(combChk>>16);
  
        rsp = -1;   // avoid dead loop
        retry = 0;
        while(retry < 3){
            tlsc6x_write_bytes_u16addr(g_tlsc6x_client, MTK_TXRX_BUF, psrc, len);
            m_rsp.id = 0;
            rsp = tlsc6x_cmd_send(&m_cmd, &m_rsp);
            if(rsp < 0){
                if((0x05==m_rsp.d0) && (0x09==m_rsp.cc)){ // fotal error
                    break;
                }else{  // some error, retry
                    retry++;
                }
            }else{
                left -= len;
                adr += len;
                psrc += len;
                break;
            }
        }
        
        if(rsp < 0){
            break;
        }       
    }

    return (!left)?0:-1;
}

static int is_valid_cfg_data(u16* ptcfg)
{
    if(NULL == ptcfg){
        return 0;
    }

    if((u8)((ptcfg[53]>>1)&0x7f) != (u8)(g_tlsc6x_client->addr)){
        return 0;
    }

    if(tlsc6x_checksum_u16(ptcfg, 204)){
        return 0;
    }

    return 1;
}
#ifdef TLSC_AUTO_UPGRADE
static int tlsc6x_tpcfg_ver_comp(unsigned short * ptcfg)
{
    unsigned int u32tmp;
    unsigned short vnow, vbuild;

    if(0 == g_tlsc6x_cfg_ver){  // no available version information
        return 0;
    }

    if(0 == is_valid_cfg_data(ptcfg)){
        return 0;
    } 

    u32tmp = ptcfg[1];
    u32tmp = (u32tmp<<16)|ptcfg[0];
    if((g_tlsc6x_cfg_ver&0x3ffffff) != (u32tmp&0x3ffffff)){//
        return 0;
    }

    vnow = (g_tlsc6x_cfg_ver>>26)&0x3f;
    vbuild = (u32tmp>>26)&0x3f;
    if(vbuild <= vnow){
        return 0;
    }

    return 1;
}
#endif
static int tlsc6x_tpcfg_ver_comp_weak(unsigned short * ptcfg)
{
    unsigned int u32tmp;

    if(0 == g_tlsc6x_cfg_ver){  // no available version information
        return 0;
    }

    if(0 == is_valid_cfg_data(ptcfg)){
        return 0;
    } 

    u32tmp = ptcfg[1];
    u32tmp = (u32tmp<<16)|ptcfg[0];
    if((g_tlsc6x_cfg_ver&0x3ffffff) != (u32tmp&0x3ffffff)){//
        return 0;
    }

    if(g_tlsc6x_cfg_ver == u32tmp){
        return 0;
    }

    return 1;
}

// 0 error
// 0x7f80 no data
static u16 tlsx6x_find_last_cfg(void)
{
    unsigned short addr, check;
    
    addr = 0x7f80-256;
    while(addr > 0x6000){   // 0x6080
        check = 0;
        if(tlsc6x_read_burn_space((u8*)&check, addr, 2)){
            addr = 0;
            goto exit;
        }
        if(0xffff == check){
            break;
        }
        addr -= 256;
    }
    
    addr += 256;

exit:

    return addr;
}
/*
static int find_last_valid_burn_cfg(u16* ptcfg)
{
    unsigned short addr;

    if(tlsc6x_download_ramcode(fw_burn_bin, sizeof(fw_burn_bin))){
        return -1;
    }

    addr = tlsx6x_find_last_cfg();
    if((0==addr) || (0x7f80==addr)){
        return -1;
    }

    flag_last_error = 0;
    while(addr <= 0x7e80){
        if(tlsc6x_read_burn_space((u8*)ptcfg, addr, 204)){
            addr = 0x7f80;  // force error
            break;
        }
        if(0 == is_valid_cfg_data(ptcfg)){
            ++flag_last_error;
            addr+=256;
        }else{
            break;
        }
    }

    return (addr<=0x7e80)?0:-1;

}
*/
// NOT a public function, only one caller!!!
static void tlsc6x_tp_cfg_version(void)
{
    unsigned int tmp;

    if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0xd6e0, (u8*)&tmp, 4)){
        return;
    }
    g_tlsc6x_cfg_ver = tmp;

    if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0xd6e0+(53*2), (u8*)&tmp, 2)){
        return;
    }
    g_tlsc6x_chip_code = (unsigned short)tmp;
    
    return;
}

int tlsx6x_update_burn_cfg(u16* ptcfg)
{
    u16 addr, check;
  
    if(0 == tlsc6x_tpcfg_ver_comp_weak(ptcfg)){
        printk("Tlsc6x:update error:version error!\n");
        return -1;
    } 

    if(g_tlsc6x_cfg_ver && ((u16)(g_tlsc6x_cfg_ver&0xffff) != ptcfg[0])){
        return -1;    
    }

    if(tlsc6x_download_ramcode(fw_burn_bin, sizeof(fw_burn_bin))){
        printk("Tlsc6x:update error:ram-code error!\n");
        return -1;
    }

    addr = tlsx6x_find_last_cfg();
    if((addr<=0x6180) || (0x7f80==addr)){
        printk("Tlsc6x:update error:time limit!\n");
        return -1;
    }

    addr = addr-256;

    // pre-check
    check = 0;
    if(tlsc6x_read_burn_space((unsigned char*)&check, addr-256, 2)){
        printk("Tlsc6x:update error:pre-read error!\n");
        return -1;
    }
    if(0xffff != check){
        printk("Tlsc6x:update error:pre-read limit!\n");
        return -1;
    }     
   
    if(tlsc6x_write_burn_space((unsigned char*)ptcfg, addr, 204)){
        printk("Tlsc6x:update fail!\n");
        return -1;
    }

    printk("Tlsc6x:update pass!\n");    

    memcpy(tl_target_cfg, ptcfg, 204);
    g_tlsc6x_cfg_ver = (ptcfg[1]<<16)|ptcfg[0];

    return 0;
}


// NOTE:caller guarantee the legitimacy
//download tp-cfg.
int tlsx6x_update_running_cfg(u16* ptcfg)
{
    unsigned int retry;
    unsigned int tmp[2];

    if(0 == is_valid_cfg_data(ptcfg)){
        return -1;
    }
    if(tlsc6x_set_dd_mode()){
        return -1;
    }

    if(tlsc6x_bulk_down_check((unsigned char*)ptcfg, 0xd7e0, 204)){   // stop mcu
        goto exit;
    }

    tmp[0] = 0x6798;
    tmp[1] = 0xcd3500ff;

    retry = 0;
    while(++retry < 3){
        if(tlsc6x_write_bytes_u16addr(g_tlsc6x_client, 0xdf10, (u8*)&tmp[0], 8)){
            msleep(5);
            continue;
        }
        break;
    }

    // write error? don't care
    retry = 0;
    while(++retry < 5){
        msleep(10);
        tmp[0] = 0;
        tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0xdf16, (u8*)&tmp[0], 1);
        if(0x30 == tmp[0]){
            break;
        }
    }

exit:
    tlsc6x_set_nor_mode();
    memcpy(tl_target_cfg, ptcfg, 204);
    return 0;
}
#ifdef TLSC_AUTO_UPGRADE
static int tlsx6x_get_running_cfg(unsigned short * ptcfg)
{
    int retry, err_type;

    retry = 0;
    err_type = 0;

    while(++retry < 5){
        err_type = 0;
        if(tlsc6x_read_bytes_u16addr(g_tlsc6x_client, 0xd6e0, (u8*)ptcfg, 204)){
            msleep(20);
            err_type = 2;   // i2c error  
            continue;
        }

        if(0==is_valid_cfg_data(ptcfg)){
            tlsc6x_set_dd_mode();
            err_type = 1;   // data error or no data
            msleep(20);
            continue;
        }
        break;
    }
    
    return err_type;

}
#endif

// return :0->no hw resetting needed
// else -> caller do hw resettting
int tlsc6x_auto_upgrade_buidin(void)
{
#ifdef TLSC_AUTO_UPGRADE
    unsigned int u32tmp, k;
//    unsigned short * ptcfg;

    if(0 == g_tlsc6x_cfg_ver){  // no available version information
        printk("Tlsc6x:no current version information!\n");
        return 0;
    }

    new_idx_active = -1;

    for(k=0; k<sizeof(buidIn_tls_tp_lut)/(102*2);k++){
        if(1 == tlsc6x_tpcfg_ver_comp(buidIn_tls_tp_lut[k])){
            new_idx_active = k;
            break;
        }
    }

    if(new_idx_active < 0){
        printk("Tlsc6x:auto update skip:no updated version!\n");
        return -1;
    }

    if(tlsc6x_set_dd_mode()){
        printk("Tlsc6x:auto update error:can't control hw mode!\n");
        return -1;
    }

    if(tlsx6x_get_running_cfg(tl_buf_tmpcfg)){
        printk("Tlsc6x:auto update error:can't get current cfg!\n");
        tlsc6x_set_nor_mode();
        return -2;
    }

    u32tmp = tl_buf_tmpcfg[1];
    u32tmp = (u32tmp<<16)|tl_buf_tmpcfg[0];

    if(u32tmp != g_tlsc6x_cfg_ver){
        g_tlsc6x_cfg_ver = u32tmp;
        tlsc6x_set_nor_mode();
        printk("Tlsc6x:auto update error:confused version!\n");
        return -3;
    }

    if(0==tlsx6x_update_burn_cfg(buidIn_tls_tp_lut[new_idx_active])){
        printk("Tlsc6x:auto update pass!\n");
    }else{
        printk("Tlsc6x:auto update fail!\n");
    }

    return 1;   // need hw reset
#else
    printk("Tlsc6x:auto update error:no build-in file!\n");
    return 0;   // no hw resetting needed
#endif
    
}

int tlsc6x_load_gesture_binlib(void)
{
	int ret;
	ret = tlsc6x_download_ramcode(tlsc6x_gesture_binlib, sizeof(tlsc6x_gesture_binlib));
    if(ret){
        printk("Tlsc6x:load gesture binlib error!\n");
        return -1;
    }
    return 0;
}

#if (defined(TLSC_AUTO_UPGRADE)) && (defined(TLSC_FORCE_UPGRADE))
// NOTE: open this function carefully!!!!
void tlsc6x_data_crash_deal(void)
{
    unsigned int u32tmp;

    new_idx_active = 1; // give the index what you want update, eg. 0,1,2

    if((sizeof(buidIn_tls_tp_lut)/(102*2)) < (1+new_idx_active)){
        printk("Tlsc6x:force update, no data!\n");
        return;
    }

    if(tlsc6x_set_dd_mode()){
        printk("Tlsc6x:auto update error:can't control hw mode!\n");
        return;
    }

    if(1 == tlsx6x_get_running_cfg(tl_buf_tmpcfg)){
        if(1 == tlsx6x_get_running_cfg(tl_buf_tmpcfg)){ // double check
            u32tmp = buidIn_tls_tp_lut[new_idx_active][1];
            u32tmp = (u32tmp<<16)|buidIn_tls_tp_lut[new_idx_active][0];
            g_tlsc6x_cfg_ver = u32tmp&0x3ffffff;
            if(0==tlsx6x_update_burn_cfg(buidIn_tls_tp_lut[new_idx_active])){
                g_tlsc6x_cfg_ver = u32tmp;
                printk("Tlsc6x:force update pass!\n");
            }else{
                g_tlsc6x_cfg_ver = 0;
                printk("Tlsc6x:force update fail!\n");
            }
        }
    }

    return;
}
#endif

// 1:telink module
int tlsc6x_tp_dect(struct i2c_client * client)
{
    g_tlsc6x_client = client;
	
    if(tlsc6x_set_dd_mode()){
        return 0;
    }

    tlsc6x_tp_cfg_version();    // MUST: call this function there!!!

    tlsc6x_set_nor_mode();

    return 1;
}

