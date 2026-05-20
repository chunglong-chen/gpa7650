#ifndef __PSC700_CONF_H__
#define __PSC700_CONF_H__

//=================================================================
//  PSC700 Version
//      20250725 V03.01 - Bob_Su
//          1.Down scale image resolution from 1008x760 to 640x480.
//          2.Solve the issue where flash LED behaves incorrectly after powering on.
//      20250725 V03.00 - Bob_Su
//          1.Add IMX577 support resolution 1012x760@120fps and 1012x760@50fps
//          2.Add IMX577 resolution control in psc700_conf.h
//          3.Add liveview resolution control in psc700_conf.h
//      20250723 V02.05 - Bob_Su
//          1.Add IMX577 VGA@50fps to IMX577 driver.
//      20250723 V02.04 - Bob_Su
//          1.Solve the issue where the camera didn't send liveview data to PC via UVC.
//          2.Add fixed AE and AWB to IMX577 driver.
//      20250711 V02.03 - Bob_Su
//          1.Add IMX577 calibration & preference table
//      20250630 V02.02 - Bob_Su
//          1.Solve the issue with "CAP" command that the flash led on time is wrong when input data contains CRLF.
//      20250627 V02.01 - Bob_Su
//          1.Disable MIPI_0 interface to solve the issue with flash led control by IOC9/10.
//      20250626 V02.00 - Bob_Su
//          1.Create a motor task for MIIS to achieve motor control and post position.
//          1.Add strobe signal detection
//          2.Add flash LED control
//          3.Add parameters in "CAP" command to control flash LED delay time and turn-on time.
//          4.Add "GIM" command to get specified captured image.
//      20250624 V01.01 - Bob_Su
//          1.Modify the vendor class from EP1 and EP2 to EP8 and EP9 to solve the issue of sending 12M data loss through USB after closing the UVC.
//=================================================================
#define PSC700_VER                                      "V03.01"
//=================================================================
//  IMX577 Configurations
//      PSC700_RES_VGA_120FPS   :  640x480@120fps
//      PSC700_RES_VGA_50FPS    :  640x480@50fps
//      PSC700_RES_760_120FPS   :  1012x760@120fps
//      PSC700_RES_760_50FPS    :  1012x760@50fps
//      PSC700_RES_FUL_25FPS    :  4056x3040@25fps
//=================================================================
#define PSC700_IMX577_RES_VGA_120FPS            0
#define PSC700_IMX577_RES_VGA_50FPS             1
#define PSC700_IMX577_RES_760_120FPS            2
#define PSC700_IMX577_RES_760_50FPS             3
#define PSC700_IMX577_RES_FUL_25FPS             4

#define PSC700_IMX577_RES                       PSC700_IMX577_RES_760_120FPS
//=================================================================
//  Liveview Configurations
//      Width               :	PSC700_LIVEVIEW_WIDTH
//      Height              :	PSC700_LIVEVIEW_HEIGHT
//      Drop Number         :   3   (F1(Drop)->F2(Drop)->F3(Drop)->F4(Liveview)->F5(Drop)......)
//      Collection Number   :   30
//=================================================================
#if (PSC700_IMX577_RES == PSC700_IMX577_RES_760_120FPS) || (PSC700_IMX577_RES == PSC700_IMX577_RES_760_50FPS)
#define PSC700_LIVEVIEW_WIDTH                   640    //Width must be a multiple of 16
#define PSC700_LIVEVIEW_HEIGHT                  480
#else
#define PSC700_LIVEVIEW_WIDTH                   640
#define PSC700_LIVEVIEW_HEIGHT                  480
#endif
#define PSC700_LIVEVIEW_DROP_NUM                3
#define PSC700_LIVEVIEW_COL_NUM                 31

#endif //__PSC700_CONF_H__
