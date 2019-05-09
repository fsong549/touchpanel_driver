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

#ifndef __tlsc6x_main_h__  
#define __tlsc6x_main_h__

//#define TP_PROXIMITY_SENSOR  // tp-prox sensor, close:undef
#define TLSC_APK_DEBUG      // apk debugger, close:undef
//#define TLSC_AUTO_UPGRADE    // update build-in version, close:undef
#define TLSC_ESD_HELPER_EN  // esd helper, close:undef
//#define TLSC_FORCE_UPGRADE  // force update, close:undef, only used for data crash

#ifdef CONFIG_CTP_GESTURE_SUPPORT
#define TP_GESTRUE
#endif

#ifdef CONFIG_CTP_PROXIMITY_SUPPORT
#define TP_PROXIMITY_SENSOR
#endif

extern unsigned int g_tlsc6x_cfg_ver;
extern unsigned short g_tlsc6x_chip_code;
extern int tlsc6x_tp_dect(struct i2c_client * client); 
extern int tlsc6x_auto_upgrade_buidin(void);
extern int tlsc6x_load_gesture_binlib(void);
extern int tlsx6x_update_running_cfg(u16* ptcfg);
extern int tlsx6x_update_burn_cfg(u16* ptcfg);
extern void tlsc6x_data_crash_deal(void);

extern int tlsc6x_i2c_Read(struct i2c_client *client, char *writebuf,int writelen, char *readbuf, int readlen);
extern int tlsc6x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);


#endif
