/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9D115 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/gc0328.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <media/nvc.h>
#define DRIVER_VERSION "0.0.1"
#define DEBUG 1

/** Macro for determining the size of an array */
#define KH_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

enum{
	GC0308,
	GC0328
};

struct sensor_reg {
	u8 addr;
	u8 val;
};

struct sensor_info {
	int mode;
	int current_wb;
	int current_exposure;
	struct i2c_client *i2c_client;
	struct yuv_sensor_platform_data *pdata;
};

static struct sensor_info *gc0328_sensor_info;
static struct sensor_reg gc0308_mode_640x480[] = {

	{0xfe,0x80},    	
	{0xfe,0x00},     // set page0
	{0xd2,0x10},     // close AEC
	{0x22,0x55},     // close AWB
	{0x03,0x01},                                    
	{0x04,0x2c},                                    
	{0x5a,0x56},  
	{0x5b,0x40},  
	{0x5c,0x4a},  			
	{0x22,0x57},     // Open AWB
	{0x0f,0x00},  
	{0x01,0x6a},    //6a  xae
	{0x02,0x50},  
	{0x0f,0x00},  
															
	{0xe2,0x00},     //anti-flicker step [11:8]
	{0xe3,0x96},     //anti-flicker step [7:0]
	{0xe4,0x02},  
	{0xe5,0x58},  //25FPS
	{0xe6,0x03},  
	{0xe7,0x84},  
	{0xe8,0x05},  
	{0xe9,0xdc},  
	{0xea,0x09},  
	{0xeb,0xd0},  
	{0xec,0x20},    
	{0x05,0x00},                                    
	{0x06,0x00},                                    
	{0x07,0x00},                                    
	{0x08,0x00},                                    
	{0x09,0x01},                                    
	{0x0a,0xe8},                                    
	{0x0b,0x02},                                    
	{0x0c,0x88},                                    
	{0x0d,0x02},                                    
	{0x0e,0x02},                                    
	{0x10,0x22},                                    
	{0x11,0xfd},                                    
	{0x12,0x2a},                                    
	{0x13,0x00},                                    
	{0x14,0x10},                                  
	{0x15,0x0a},                                    
	{0x16,0x05},                                    
	{0x17,0x01},                                    
	{0x18,0x44},                                    
	{0x19,0x44},                                    
	{0x1a,0x1e},                                    
	{0x1b,0x00},                                    
	{0x1c,0xc1},                                    
	{0x1d,0x08},                                    
	{0x1e,0x60},                                    
	{0x1f,0x17},                                    
															  
	{0x20,0xff},                                    
	{0x21,0xf8},                                    
	{0x22,0x57},                                    
	{0x24,0xa2},                                    
	{0x25,0x0f},                                    
																		 
		//output sync_mode                                               
	{0x26,0x03},     //0x03  20101016 zhj                                 
	{0x2f,0x01},                                    
	{0x30,0xf7},                                    
	{0x31,0x50},  
	{0x32,0x00},  
	{0x39,0x04},  
	{0x3a,0x18},  
	{0x3b,0x20},                                    
	{0x3c,0x00},                                    
	{0x3d,0x00},                                    
	{0x3e,0x00},                                    
	{0x3f,0x00},                                    
	{0x50,0x10},                                    
	{0x53,0x82},                                    
	{0x54,0x80},                                    
	{0x55,0x80},                                    
	{0x56,0x82},                                    
	{0x8b,0x40},                                    
	{0x8c,0x40},                                    
	{0x8d,0x40},                                    
	{0x8e,0x2e},                                    
	{0x8f,0x2e},                                    
	{0x90,0x2e},                                    
	{0x91,0x3c},                                    
	{0x92,0x50},                                    
	{0x5d,0x12},                                    
	{0x5e,0x1a},                                    
	{0x5f,0x24},                                    
	{0x60,0x07},                                    
	{0x61,0x15},                                    
	{0x62,0x08},                                    
	{0x64,0x03},                                    
	{0x66,0xe8},                                    
	{0x67,0x86},                                    
	{0x68,0xa2},                                    
	{0x69,0x18},                                    
	{0x6a,0x0f},                                    
	{0x6b,0x00},                                    
	{0x6c,0x5f},                                    
	{0x6d,0x8f},                                    
	{0x6e,0x55},                                    
	{0x6f,0x38},                                    
	{0x70,0x15},                                    
	{0x71,0x33},                                    
	{0x72,0xdc},                                    
	{0x73,0x80},                                    
	{0x74,0x02},                                    
	{0x75,0x3f},                                    
	{0x76,0x02},                                    
	{0x77,0x36},                                    
	{0x78,0x88},                                    
	{0x79,0x81},                                    
	{0x7a,0x81},                                    
	{0x7b,0x22},                                    
	{0x7c,0xff},                                    
	{0x93,0x48},                                    
	{0x94,0x00},                                    
	{0x95,0x05},                                    
	{0x96,0xe8},                                    
	{0x97,0x40},                                    
	{0x98,0xf0},                                    
	{0xb1,0x38},                                    
	{0xb2,0x38},                                    
	{0xbd,0x38},                                    
	{0xbe,0x36},                                    
	{0xd0,0xc9},                                    
	{0xd1,0x90},                                    
	{0xd3,0x80},                                    
	{0xd5,0xf2},                                    
	{0xd6,0x16},                                    
	{0xdb,0x92},                                    
	{0xdc,0xa5},                                    
	{0xdf,0x23},                                    
	{0xd9,0x00},                                    
	{0xda,0x00},                                    
	{0xe0,0x09},                                    
	{0xed,0x01},                                    
	{0xee,0xb0},                                    
	{0xef,0x50},                                    
	{0x80,0x03},                                    
	{0x80,0x03},                                    
	{0x9F,0x10},                                    
	{0xA0,0x20},                                    
	{0xA1,0x38},                                    
	{0xA2,0x4E},                                    
	{0xA3,0x63},                                    
	{0xA4,0x76},                                    
	{0xA5,0x87},                                    
	{0xA6,0xA2},                                    
	{0xA7,0xB8},                                    
	{0xA8,0xCA},                                    
	{0xA9,0xD8},                                    
	{0xAA,0xE3},                                    
	{0xAB,0xEB},                                    
	{0xAC,0xF0},                                    
	{0xAD,0xF8},                                    
	{0xAE,0xFD},                                    
	{0xAF,0xFF},                                    
	{0xc0,0x00},                                    
	{0xc1,0x10},                                    
	{0xc2,0x1C},                                    
	{0xc3,0x30},                                    
	{0xc4,0x43},                                    
	{0xc5,0x54},                                    
	{0xc6,0x65},                                    
	{0xc7,0x75},                                    
	{0xc8,0x93},                                    
	{0xc9,0xB0},                                    
	{0xca,0xCB},                                    
	{0xcb,0xE6},                                    
	{0xcc,0xFF},                                    
	{0xf0,0x02},                                    
	{0xf1,0x01},                                    
	{0xf2,0x01},                                    
	{0xf3,0x30},                                    
	{0xf9,0x9f},                                    
	{0xfa,0x78},                                    
																 
	//---------------------------------------------------------------
	{0xfe,0x01},  // set page1                                            
															 
	{0x00,0xf5},                                    
	{0x02,0x1a},                                    
	{0x0a,0xa0},                                    
	{0x0b,0x60},                                    
	{0x0c,0x08},                                    
	{0x0e,0x4c},                                    
	{0x0f,0x39},                                    
	{0x11,0x3f},                                    
	{0x12,0x72},                                    
	{0x13,0x13},                                    
	{0x14,0x42},                                    
	{0x15,0x43},                                    
	{0x16,0xc2},                                    
	{0x17,0xa8},                                    
	{0x18,0x18},                                    
	{0x19,0x40},                                    
	{0x1a,0xd0},                                    
	{0x1b,0xf5},                                    
	{0x70,0x40},                                    
	{0x71,0x58},                                    
	{0x72,0x30},                                    
	{0x73,0x48},                                    
	{0x74,0x20},                                    
	{0x75,0x60},                                    
	{0x77,0x20},                                    
	{0x78,0x32},                                    
	{0x30,0x03},                                    
	{0x31,0x40},                                    
	{0x32,0xe0},                                    
	{0x33,0xe0},                                    
	{0x34,0xe0},                                    
	{0x35,0xb0},                                    
	{0x36,0xc0},                                    
	{0x37,0xc0},                                    
	{0x38,0x04},                                    
	{0x39,0x09},                                    
	{0x3a,0x12},                                    
	{0x3b,0x1C},                                    
	{0x3c,0x28},                                    
	{0x3d,0x31},                                    
	{0x3e,0x44},                                    
	{0x3f,0x57},                                    
	{0x40,0x6C},                                    
	{0x41,0x81},                                    
	{0x42,0x94},                                    
	{0x43,0xA7},                                    
	{0x44,0xB8},                                    
	{0x45,0xD6},                                    
	{0x46,0xEE},                                    
	{0x47,0x0d},                                    
	{0xfe,0x00},   // set page0
		 
		//-----------Update the registers 2010/07/06-------------//
		//Registers of Page0
	{0xfe,0x00},   // set page0
	{0x10,0x26},                                   
	{0x11,0x0d},    // fd,modified by mormo 2010/07/06                               
	{0x1a,0x2a},    // 1e,modified by mormo 2010/07/06                                  

	{0x1c,0x49},   // c1,modified by mormo 2010/07/06                                 
	{0x1d,0x9a},   // 08,modified by mormo 2010/07/06                                 
	{0x1e,0x61},   // 60,modified by mormo 2010/07/06                                 

	{0x3a,0x20},  

	{0x50,0x14},    // 10,modified by mormo 2010/07/06                               
	{0x53,0x80},                                    
	{0x56,0x80},  

	{0x8b,0x20},   //LSC                                 
	{0x8c,0x20},                                    
	{0x8d,0x20},                                    
	{0x8e,0x14},                                    
	{0x8f,0x10},                                    
	{0x90,0x14},                                    

	{0x94,0x02},                                    
	{0x95,0x07},                                    
	{0x96,0xe0},                                    

	{0xb1,0x3c},   // YCPT                                 
	{0xb2,0x3c},                                    
	{0xb3,0x40},  
	{0xb6,0xe0},  

	{0xd0,0xc9},   // AECT  c9,modifed by mormo 2010/07/06                                
	{0xd3,0x80},   // 80,modified by mormor 2010/07/06                           

	{0xf2,0x02},                                    
	{0xf7,0x12},  
	{0xf8,0x0a},  

		//Registers of Page1
	{0xfe,0x01},  // set page1    
	{0x02,0x20},  
	{0x04,0x10},  
	{0x05,0x08},  
	{0x06,0x20},  
	{0x08,0x0a},  

	{0x0e,0x44},                                    
	{0x0f,0x32},  
	{0x10,0x41},                                    
	{0x11,0x37},                                    
	{0x12,0x22},                                    
	{0x13,0x19},                                    
	{0x14,0x44},                                    
	{0x15,0x44},    

	{0x19,0x50},                                    
	{0x1a,0xd8},   

	{0x32,0x10},   

	{0x35,0x00},                                    
	{0x36,0x80},                                    
	{0x37,0x00},   
		//-----------Update the registers end---------//


	{0xfe,0x00},   // set page0
	{0xd2,0x90},  


		//-----------GAMMA Select(3)---------------//
	{0x9F,0x10},  
	{0xA0,0x20},  
	{0xA1,0x38},  
	{0xA2,0x4E},  
	{0xA3,0x63},  
	{0xA4,0x76},  
	{0xA5,0x87},  
	{0xA6,0xA2},  
	{0xA7,0xB8},  
	{0xA8,0xCA},  
	{0xA9,0xD8},  
	{0xAA,0xE3},  
	{0xAB,0xEB},  
	{0xAC,0xF0},  
	{0xAD,0xF8},  
	{0xAE,0xFD},  
	{0xAF,0xFF},  



{SENSOR_TABLE_END, 0x00}
};
static struct sensor_reg gc0328_mode_640x480[] = {

	{0xfe , 0x80},    
	{0xfe , 0x80},    
	{0xfc , 0x16},     
	{0xfc , 0x16},     
	{0xfc , 0x16},     
	{0xfc , 0x16},     
	
	{0xfe , 0x00},   
	{0x4f , 0x00},  
	{0x42 , 0x00},  
	{0x03 , 0x00},  
	{0x04 , 0xc0},  
	{0x77 , 0x62},  
	{0x78 , 0x40},  
	{0x79 , 0x4d},  
	
	
	{0xfe , 0x01},    
	{0x4f , 0x00},    
	{0x4c , 0x01},     
	{0xfe , 0x00},    
	//////////////////////////////
	 ///////////AWB///////////
	////////////////////////////////
#if 1	
	{0xfe , 0x01},
	{0x51 , 0x80}, //20
	{0x52 , 0x12}, //16 1f
	{0x53 , 0x80}, //40
	{0x54 , 0x60}, //9f
	{0x55 , 0x04}, //01
	{0x56 , 0x0e}, //00
	{0x5b , 0x02},//02
	{0x61 , 0xdc}, //R2G_stand0[70]
	{0x62 , 0xca},  //B2G_stand0[70]

	{0x70 , 0xf5},
	{0x71 , 0x0a},
	{0x72 , 0x18},  // y2c
	{0x73 , 0x30}, //28 20  AWB_C_inter
	{0x74 , 0x58}, //40 20   AWB_C_max
	{0x7c , 0x71}, //AWB speed,AWB margin
	{0x7d , 0x00}, //10 AWB every N
	{0x76 , 0x8f}, //move mode en,Move mode TH
	{0x79 , 0x00}, //00
	{0x4f , 0x00},
	{0x4d , 0x34},
	{0x4e , 0x04},
	{0x4e , 0x02},
	{0x4e , 0x02},
	{0x4d , 0x43},
	{0x4e , 0x04},
	{0x4e , 0x04},//D50
	{0x4e , 0x02}, //d65 add
	{0x4e , 0x02}, //d65 add
	{0x4d , 0x53},
	{0x4e , 0x08},  //cwf
	{0x4e , 0x04},
	{0x4e , 0x02},  //D65
	{0x4e , 0x02},  //D65
	{0x4d , 0x63},
	{0x4e , 0x10},  //tl84
	{0x4d , 0x72},
	{0x4e , 0x20},
	{0x4e , 0x20},
	{0x4d , 0x82},
	{0x4e , 0x20},  //A 
	{0x4e , 0x20},
	{0x4d , 0x92},   
	{0x4e , 0x20},  //A
	{0x4e , 0x20},
	{0x4d , 0xa0},
	{0x4e , 0x40},  //  H
	{0x4e , 0x40},  
	{0x4e , 0x40},
	{0x4f , 0x01},    
	{0x50 , 0x84}, //80
	{0xfe , 0x00}, //page0

#else   //  mtk  awb
	{0xfe , 0x01},   
	{0x51 , 0x80},   
	{0x52 , 0x12},   
	{0x53 , 0x80},   
	{0x54 , 0x60},   
	{0x55 , 0x01},   
	{0x56 , 0x06},   
	{0x5b , 0x02},   
	{0x61 , 0xdc},   
	{0x62 , 0xdc},   
	{0x7c , 0x71},   
	{0x7d , 0x00},   
	{0x76 , 0x00},   
	{0x79 , 0x20},   
	{0x7b , 0x00},   
	{0x70 , 0xFF}, 
	{0x71 , 0x00},   
	{0x72 , 0x10},   
	{0x73 , 0x40},   
	{0x74 , 0x40},   
	////AWB//
	{0x50 , 0x00},    
	{0xfe , 0x01},    
	{0x4f , 0x00},     
	{0x4c , 0x01},   
	{0x4f , 0x00},   
	{0x4f , 0x00},  
	{0x4f , 0x00},  
	{0x4d , 0x36},  
	{0x4e , 0x02},  
	{0x4d , 0x46},  
	{0x4e , 0x02},  
	{0x4e , 0x02},  
	{0x4d , 0x53},  
	{0x4e , 0x08},  
	{0x4e , 0x04},  
	{0x4e , 0x04},  
	{0x4d , 0x63},  
	{0x4e , 0x08},  
	{0x4e , 0x08},  
	{0x4d , 0x82},  
	{0x4e , 0x20},  
	{0x4e , 0x20},  
	{0x4d , 0x92},  
	{0x4e , 0x40},  
	{0x4d , 0xa2},  
	{0x4e , 0x40},  
	{0x4f , 0x01},  
	
	{0x50 , 0x88},    
	{0xfe , 0x00},  

#endif
	////////////////////////////////////////////////
	////////////     BLK      //////////////////////
	////////////////////////////////////////////////
	{0x27 , 0x00},   
	{0x2a , 0x40},  
	{0x2b , 0x40},  
	{0x2c , 0x40},  
	{0x2d , 0x40},  
	
	
	//////////////////////////////////////////////
	////////// page  0    ////////////////////////
	//////////////////////////////////////////////
	{0xfe , 0x00},   
	
	{0x0d , 0x01},    
	{0x0e , 0xe8},    
	{0x0f , 0x02},    
	{0x10 , 0x88},    
	{0x09 , 0x00},    
	{0x0a , 0x00},    
	{0x0b , 0x00},    
	{0x0c , 0x00},    
	{0x16 , 0x00},    
	{0x17 , 0x14},    
	{0x18 , 0x0e},    
	{0x19 , 0x06},    
	
	{0x1b , 0x48},    
	{0x1f , 0xC8},    
	{0x20 , 0x01},    
	{0x21 , 0x78},    
	{0x22 , 0xb0},    
	{0x23 , 0x06},    
	{0x24 , 0x11},    
	{0x26 , 0x00},    
	
	{0x50 , 0x01}, //crop mode
	                
	//global gain for range 
	{0x70 , 0x85},   
	
	
	////////////////////////////////////////////////
	////////////     block enable      /////////////
	////////////////////////////////////////////////
	{0x40 , 0x7f},   
	{0x41 , 0x24},   
	{0x42 , 0xff},
	{0x45 , 0x00},   
	{0x44 , 0x02},   // yuv format
	{0x46 , 0x03},   // hs  vs
	
	{0x4b , 0x01},   
	{0x50 , 0x01},  
	
	//DN & EEINTP
	{0x7e , 0x0a},    
	{0x7f , 0x03},    
	{0x81 , 0x15},    
	{0x82 , 0x85},    
	{0x83 , 0x02},    
	{0x84 , 0xe5},    
	{0x90 , 0xac},    
	{0x92 , 0x02},    
	{0x94 , 0x02},    
	{0x95 , 0x54},    
	
	///////YCP
	{0xd1 , 0x32},
	{0xd2 , 0x32},
	{0xdd , 0x58},
	{0xde , 0x36},
	{0xe4 , 0x88},
	{0xe5 , 0x40},    
	{0xd7 , 0x0e},    
	                      
	///////////////////////////// 
	//////////////// GAMMA ////// 
	///////////////////////////// 
	//rgb gamma                  
	{0xfe , 0x00},
	{0xbf , 0x08},
	{0xc0 , 0x10},
	{0xc1 , 0x22},
	{0xc2 , 0x32},
	{0xc3 , 0x43},
	{0xc4 , 0x50},
	{0xc5 , 0x5e},
	{0xc6 , 0x78},
	{0xc7 , 0x90},
	{0xc8 , 0xa6},
	{0xc9 , 0xb9},
	{0xca , 0xc9},
	{0xcb , 0xd6},
	{0xcc , 0xe0},
	{0xcd , 0xee},
	{0xce , 0xf8},
	{0xcf , 0xff},
#if 0
//case GC0328_RGB_Gamma_m1:						//smallest gamma curve
	{0xfe , 0x00},
	{0xbf , 0x06},
	{0xc0 , 0x12},
	{0xc1 , 0x22},
	{0xc2 , 0x35},
	{0xc3 , 0x4b},
	{0xc4 , 0x5f},
	{0xc5 , 0x72},
	{0xc6 , 0x8d},
	{0xc7 , 0xa4},
	{0xc8 , 0xb8},
	{0xc9 , 0xc8},
	{0xca , 0xd4},
	{0xcb , 0xde},
	{0xcc , 0xe6},
	{0xcd , 0xf1},
	{0xce , 0xf8},
	{0xcf , 0xfd},
//case GC0328_RGB_Gamma_m2:
	{0xBF , 0x08},
	{0xc0 , 0x0F},
	{0xc1 , 0x21},
	{0xc2 , 0x32},
	{0xc3 , 0x43},
	{0xc4 , 0x50},
	{0xc5 , 0x5E},
	{0xc6 , 0x78},
	{0xc7 , 0x90},
	{0xc8 , 0xA6},
	{0xc9 , 0xB9},
	{0xcA , 0xC9},
	{0xcB , 0xD6},
	{0xcC , 0xE0},
	{0xcD , 0xEE},
	{0xcE , 0xF8},
	{0xcF , 0xFF},
//case GC0328_RGB_Gamma_m3:			
	{0xBF , 0x0B},
	{0xc0 , 0x16},
	{0xc1 , 0x29},
	{0xc2 , 0x3C},
	{0xc3 , 0x4F},
	{0xc4 , 0x5F},
	{0xc5 , 0x6F},
	{0xc6 , 0x8A},
	{0xc7 , 0x9F},
	{0xc8 , 0xB4},
	{0xc9 , 0xC6},
	{0xcA , 0xD3},
	{0xcB , 0xDD},
	{0xcC , 0xE5},
	{0xcD , 0xF1},
	{0xcE , 0xFA},
	{0xcF , 0xFF},
//case GC0328_RGB_Gamma_m4:
	{0xBF , 0x0E},
	{0xc0 , 0x1C},
	{0xc1 , 0x34},
	{0xc2 , 0x48},
	{0xc3 , 0x5A},
	{0xc4 , 0x6B},
	{0xc5 , 0x7B},
	{0xc6 , 0x95},
	{0xc7 , 0xAB},
	{0xc8 , 0xBF},
	{0xc9 , 0xCE},
	{0xcA , 0xD9},
	{0xcB , 0xE4},
	{0xcC , 0xEC},
	{0xcD , 0xF7},
	{0xcE , 0xFD},
	{0xcF , 0xFF},
//case GC0328_RGB_Gamma_m5:
	{0xBF , 0x10},
	{0xc0 , 0x20},
	{0xc1 , 0x38},
	{0xc2 , 0x4E},
	{0xc3 , 0x63},
	{0xc4 , 0x76},
	{0xc5 , 0x87},
	{0xc6 , 0xA2},
	{0xc7 , 0xB8},
	{0xc8 , 0xCA},
	{0xc9 , 0xD8},
	{0xcA , 0xE3},
	{0xcB , 0xEB},
	{0xcC , 0xF0},
	{0xcD , 0xF8},
	{0xcE , 0xFD},
	{0xcF , 0xFF},
//case GC0328_RGB_Gamma_m6:										// largest gamma curve
	{0xBF , 0x14},
	{0xc0 , 0x28},
	{0xc1 , 0x44},
	{0xc2 , 0x5D},
	{0xc3 , 0x72},
	{0xc4 , 0x86},
	{0xc5 , 0x95},
	{0xc6 , 0xB1},
	{0xc7 , 0xC6},
	{0xc8 , 0xD5},
	{0xc9 , 0xE1},
	{0xcA , 0xEA},
	{0xcB , 0xF1},
	{0xcC , 0xF5},
	{0xcD , 0xFB},
	{0xcE , 0xFE},
	{0xcF , 0xFF},
//case GC0328_RGB_Gamma_night:									//Gamma for night mode
	{0xBF , 0x0B},
	{0xc0 , 0x16},
	{0xc1 , 0x29},
	{0xc2 , 0x3C},
	{0xc3 , 0x4F},
	{0xc4 , 0x5F},
	{0xc5 , 0x6F},
	{0xc6 , 0x8A},
	{0xc7 , 0x9F},
	{0xc8 , 0xB4},
	{0xc9 , 0xC6},
	{0xcA , 0xD3},
	{0xcB , 0xDD},
	{0xcC , 0xE5},
	{0xcD , 0xF1},
	{0xcE , 0xFA},
	{0xcF , 0xFF},


#endif
	///Y gamma           
	{0xfe , 0x00},    
	{0x63 , 0x00},    
	{0x64 , 0x05},    
	{0x65 , 0x0b},    
	{0x66 , 0x19},    
	{0x67 , 0x2e},    
	{0x68 , 0x40},    
	{0x69 , 0x54},    
	{0x6a , 0x66},    
	{0x6b , 0x86},    
	{0x6c , 0xa7},    
	{0x6d , 0xc6},    
	{0x6e , 0xe4},    
	{0x6f , 0xFF},
	               
	//////ASDE             
	{0xfe , 0x01},    
	{0x18 , 0x02},    
	{0xfe , 0x00},    
	{0x98 , 0x00},    
	{0x9b , 0x20},    
	{0x9c , 0x80},    
	{0xa4 , 0x10},    
	{0xa8 , 0xB0},    
	{0xaa , 0x40},    
	{0xa2 , 0x23},    
	{0xad , 0x01},    
	
	//////////////////////////////////////////////
	////////// AEC    ////////////////////////
	//////////////////////////////////////////////
	{0xfe , 0x01},   
	{0x9c , 0x02},   
	{0x08 , 0xa0},   
	{0x09 , 0xe8},   
	
	{0x10 , 0x00},  
	{0x11 , 0x11},   
	{0x12 , 0x10},   
	{0x13 , 0x80},   
	{0x15 , 0xfc},   
	{0x18 , 0x03},
	{0x21 , 0xc0},   
	{0x22 , 0x60},   
	{0x23 , 0x30},   
	{0x25 , 0x00},   
	{0x24 , 0x14},   

 	{0xfe , 0x00}, 	
 	{0x05 , 0x02}, 	
 	{0x06 , 0x2c}, 
 	{0x07 , 0x00},
 	{0x08 , 0xb8},
 
 	{0xfe , 0x01},   
 	{0x29 , 0x00},   //anti-flicker step [118]
 	{0x2a , 0x60},   //anti-flicker step [70]
 
 	{0x2b , 0x02},   //exp level 0  14.28fps
 	{0x2c , 0xa0}, 
 	{0x2d , 0x03},   //exp level 1  12.50fps
 	{0x2e , 0x00}, 
 	{0x2f , 0x03},   //exp level 2  10.00fps
 	{0x30 , 0xc0}, 
 	{0x31 , 0x05},   //exp level 3  7.14fps
 	{0x32 , 0x40}, 
 	{0x33 , 0x20}, 

	
	//////////////////////////////////////
	////////////LSC//////////////////////
	//////////////////////////////////////
	//gc0328 Alight lsc reg setting list
	////Record date: 2013-04-01 15:59:05
	{0xfe , 0x01},
	{0xc0 , 0x0d},
	{0xc1 , 0x05},
	{0xc2 , 0x00},
	{0xc6 , 0x07},
	{0xc7 , 0x03},
	{0xc8 , 0x01},
	{0xba , 0x19},
	{0xbb , 0x10},
	{0xbc , 0x0a},
	{0xb4 , 0x19},
	{0xb5 , 0x0d},
	{0xb6 , 0x09},
	{0xc3 , 0x00},
	{0xc4 , 0x00},
	{0xc5 , 0x0e},
	{0xc9 , 0x00},
	{0xca , 0x00},
	{0xcb , 0x00},
	{0xbd , 0x07},
	{0xbe , 0x00},
	{0xbf , 0x0e},
	{0xb7 , 0x09},
	{0xb8 , 0x00},
	{0xb9 , 0x0d},
	{0xa8 , 0x01},
	{0xa9 , 0x00},
	{0xaa , 0x03},
	{0xab , 0x02},
	{0xac , 0x05},
	{0xad , 0x0c},
	{0xae , 0x03},
	{0xaf , 0x00},
	{0xb0 , 0x04},
	{0xb1 , 0x04},
	{0xb2 , 0x03},
	{0xb3 , 0x08},
	{0xa4 , 0x00},
	{0xa5 , 0x00},
	{0xa6 , 0x00},
	{0xa7 , 0x00},
	{0xa1 , 0x3c},
	{0xa2 , 0x50},
	{0xfe , 0x00},
	              
	///cct       
	{0xB1 , 0x02},   
	{0xB2 , 0x02},   
	{0xB3 , 0x07},   
	{0xB4 , 0xf0},   
	{0xB5 , 0x05},   
	{0xB6 , 0xf0},   
	
	
	{0xfe , 0x00},   
	{0x27 , 0xf7},  
	{0x28 , 0x7F},   
	{0x29 , 0x20},   
	{0x33 , 0x20},   
	{0x34 , 0x20},   
	{0x35 , 0x20},   
	{0x36 , 0x20},   
	{0x32 , 0x08},   
	
	{0x47 , 0x00},   
	{0x48 , 0x00},   
	
	{0xfe , 0x01},  
	{0x79 , 0x00},  
	{0x7d , 0x00},   
	{0x50 , 0x88},   
	{0x5b , 0x04}, 
	{0x76 , 0x8f},   
	{0x80 , 0x70},
	{0x81 , 0x70},
	{0x82 , 0xb0},
	{0x70 , 0xff}, 
	{0x71 , 0x00}, 
	{0x72 , 0x10}, 
	{0x73 , 0x40}, 
	{0x74 , 0x40}, 
	
	{0xfe , 0x00},  
	{0x70 , 0x45},  
	{0x4f , 0x01},  
	{0xf1 , 0x07},  
	
	{0xf2 , 0x01}, 



{SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_WhiteB_Auto[]=
{
	{0xfe,0x00},
	{0x77,0x62},//57
	{0x78,0x40},//4
	{0x79,0x4d},//45
	{0x42,0xff},
	{SENSOR_TABLE_END, 0x00}
};
/* Cloudy Colour Temperature : 6500K - 8000K  */
static  struct sensor_reg sensor_WhiteB_Cloudy[]=
{
	{0xfe,0x00},
	{0x42,0xfd},

	{0x77,0x8c},
	{0x78,0x50},
	{0x79,0x40},
    {SENSOR_TABLE_END, 0x00}
};
/* ClearDay Colour Temperature : 5000K - 6500K  */
static  struct sensor_reg sensor_WhiteB_ClearDay[]=
{
    //Sunny
	{0xfe,0x00},
	{0x42,0xfd},

	{0x77,0x74},
	{0x78,0x52},
	{0x79,0x40},
    {SENSOR_TABLE_END, 0x00}
};
/* Office Colour Temperature : 3500K - 5000K  */
static  struct sensor_reg sensor_WhiteB_TungstenLamp1[]=
{
    //Office
    {0xfe, 0x00},
	{0x42, 0xfd},
	{0x77, 0x48},
	{0x78, 0x40},
	{0x79, 0x5c},
    {SENSOR_TABLE_END, 0x00}

};



static  struct sensor_reg sensor_Effect_Normal[] =
{
	{0x43,0x00},
    {SENSOR_TABLE_END,0x00}
};

static  struct sensor_reg sensor_Effect_WandB[] =
{
	{0x43,0x02},
	{0xda,0xd0},
	{0xdb,0x28},
    {SENSOR_TABLE_END,0x00}
};

static  struct sensor_reg sensor_Effect_Sepia[] =
{
	{0x43,0x02},
	{0xda,0xc0},
	{0xdb,0xc0},
    {SENSOR_TABLE_END,0x00}
};

static  struct sensor_reg sensor_Effect_Negative[] =
{
    //Negative
	{0x43,0x01},
    {SENSOR_TABLE_END,0x00}
};
static  struct sensor_reg sensor_Effect_Bluish[] =
{
    // Bluish
	{0x43,0x02},
	{0xda,0x50},
	{0xdb,0xe0},
    {SENSOR_TABLE_END,0x00}
};

static  struct sensor_reg sensor_Effect_Green[] =
{
    //  Greenish
	{0x43,0x02},

	{0xda,0xc0},
	{0xdb,0xc0},
    {SENSOR_TABLE_END,0x00}
};



static  struct sensor_reg sensor_Exposure0[]=
{
	{0xfe,0x00},
	{0xd5,0xe0},
	{0xfe,0x01},
	{0x13,0x70},
	{0xfe,0x00},
    {SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_Exposure1[]=
{
	{0xfe,0x00},
	{0xd5,0xf0},
	{0xfe,0x01},
	{0x13,0x78},
	{0xfe,0x00},
    {SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_Exposure2[]=
{
	{0xfe,0x00},
	{0xd5,0x00},
	{0xfe,0x01},
	{0x13,0x80},
	{0xfe,0x00},
    {SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_Exposure3[]=
{
	{0xfe,0x00},
	{0xd5,0x10},
	{0xfe,0x01},
	{0x13,0x88},
	{0xfe,0x00},
    {SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_Exposure4[]=
{
	{0xfe,0x00},
	{0xd5,0x20},
	{0xfe,0x01},
	{0x13,0x90},
	{0xfe,0x00},
    {SENSOR_TABLE_END, 0x00}
};


static  struct sensor_reg sensor_SceneAuto[] =
{
	{0xfe, 0x01},
	{0x33, 0x20},
	{0xfe, 0x00},
	{SENSOR_TABLE_END, 0x00}
};

static  struct sensor_reg sensor_SceneNight[] =
{
	{0xfe, 0x01},
	{0x33, 0x30},
	{0xfe, 0x00},
	{SENSOR_TABLE_END, 0x00}
};

enum {
	SENSOR_MODE_640x480,
};

static struct sensor_reg *gc0308_mode_table[] = {
	[SENSOR_MODE_640x480]   = gc0308_mode_640x480,
};
static struct sensor_reg *gc0328_mode_table[] = {
	[SENSOR_MODE_640x480]   = gc0328_mode_640x480,
};
static int sensor_write_reg8(struct i2c_client *client, u8 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	u8 data[2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	//data[0] = (u8) (addr >> 8);
	//data[1] = (u8) (addr & 0xff);
	//data[2] = (u8) (val & 0xff);

	data[0] = addr & 0xFF;
    	data[1] = val;
	
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("gc0328 : i2c transfer failed, retrying %x %x %d\n",
		       addr, val, err);
		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	const struct sensor_reg *next;
	int err;
    
	pr_info("gc0328 %s \n",__func__);
    next = table ;       

	for (next = table; next->addr!= SENSOR_TABLE_END; next++) {
	       err = sensor_write_reg8(client, next->addr,next->val);
		if (err){
			pr_err("%s: write  0x%x failed\n", __func__,
				next->addr);
			return err;
		}
	}
	return 0;
}
static int sensor_read_reg8_addr8(struct i2c_client *client, u8 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = data;

	/* high byte goes out first */
	//data[0] = (u8) (addr >> 8);
	data[0] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 1;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err<0)
		return -EINVAL;

        //swap(*(data+2),*(data+3)); //swap high and low byte to match table format
	memcpy(val, data+1, 1);

	return 0;
}
static int sensor_set_mode(struct sensor_info *info, struct gc0308_mode *mode)
{
	int sensor_table;
	int err;
	u8 val = 0x00;


	pr_info("%s: xres %u yres %u\n",__func__, mode->xres, mode->yres);
	 if (mode->xres == 640 && mode->yres == 480)
		sensor_table = SENSOR_MODE_640x480;
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	err = sensor_read_reg8_addr8(info->i2c_client, 0xf0,&val);
	printk("%s: read  0x%x =0x%x \n",__func__,0xf0,val);
    //check already program the sensor mode, Aptina support Context B fast switching capture mode back to preview mode
    //we don't need to re-program the sensor mode for 640x480 table
	if(val == 0x9d){
        err = sensor_write_table(info->i2c_client, gc0328_mode_table[sensor_table]);
        if (err)
            return err;
	}else{
		err = sensor_write_table(info->i2c_client, gc0308_mode_table[sensor_table]);
        	if (err)
            	return err;
	}
		
#if 0
	switch(info->current_wb){
		case YUV_Whitebalance_Auto:

			err = sensor_write_table(info->i2c_client, sensor_WhiteB_Auto);
		 break;
               case YUV_Whitebalance_Incandescent:

                 	err = sensor_write_table(info->i2c_client, sensor_WhiteB_TungstenLamp1);
                break;
                case YUV_Whitebalance_Daylight:

                 	err = sensor_write_table(info->i2c_client, sensor_WhiteB_ClearDay);
                     break;
                case YUV_Whitebalance_Fluorescent:

                 	err = sensor_write_table(info->i2c_client, sensor_WhiteB_TungstenLamp1);
                     break;
		  case YUV_Whitebalance_CloudyDaylight:

                 	err = sensor_write_table(info->i2c_client, sensor_WhiteB_Cloudy);
                     break;
                default:
                     break;
		}
	     switch(info->current_exposure)
            {
                case YUV_YUVExposure_Positive2:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure4);
                     break;
                case YUV_YUVExposure_Positive1:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure3);
                     break;
                case YUV_YUVExposure_Number0:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure2);
                     break;
                case YUV_YUVExposure_Negative1:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure1);
                     break;
		  case YUV_YUVExposure_Negative2:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure0);
                     break;
                default:
                     break;
            }
#endif
	info->mode = sensor_table;
	return 0;
}
static int gc0328_get_status(struct sensor_info *info,
		struct gc0308_status *dev_status)
{
	int err = 0;
       dev_status->status = 0;

	return err;
}

static long sensor_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct sensor_info *info = file->private_data;
    int err=0;

	pr_info("gc0328 %s\n cmd %d", __func__, cmd);
    
	switch (cmd) 
    {
    	case GC0308_IOCTL_SET_MODE:
    	{
    		struct gc0308_mode mode;
    		if (copy_from_user(&mode,
    				   (const void __user *)arg,
    				   sizeof(struct gc0308_mode))) {
    			return -EFAULT;
    		}
		
    		return sensor_set_mode(info, &mode);
    	}
        case GC0308_IOCTL_SET_COLOR_EFFECT:
        {
            u8 coloreffect;

        	if (copy_from_user(&coloreffect,
        			   (const void __user *)arg,
        			   sizeof(coloreffect))) {
        		return -EFAULT;
        	}

            switch(coloreffect)
            {
                case YUV_ColorEffect_None:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Normal);
                     break;
                case YUV_ColorEffect_Mono:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_WandB);
                     break;
                case YUV_ColorEffect_Sepia:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Sepia);
                     break;
                case YUV_ColorEffect_Negative:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Negative);
                     break;
                case YUV_ColorEffect_Solarize:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Bluish);
                     break;
                case YUV_ColorEffect_Posterize:
                 err = sensor_write_table(info->i2c_client, sensor_Effect_Green);
                     break;
                default: 	
                     break;
            }

            if (err)
    	        return err;

            return 0;
        }
        case GC0308_IOCTL_SET_WHITE_BALANCE:
        {
            u8 whitebalance;
        	if (copy_from_user(&whitebalance,
        			   (const void __user *)arg,
        			   sizeof(whitebalance))) {
        		return -EFAULT;
        	}

            switch(whitebalance)
            {
                case YUV_Whitebalance_Auto:

                     err = sensor_write_table(info->i2c_client, sensor_WhiteB_Auto);
		       info->current_wb =YUV_Whitebalance_Auto; 
                     break;
                case YUV_Whitebalance_Incandescent:

                 err = sensor_write_table(info->i2c_client, sensor_WhiteB_TungstenLamp1);
		       info->current_wb =YUV_Whitebalance_Incandescent; 
                     break;
                case YUV_Whitebalance_Daylight:
                 err = sensor_write_table(info->i2c_client, sensor_WhiteB_ClearDay);
			info->current_wb =YUV_Whitebalance_Daylight; 
                     break;
                case YUV_Whitebalance_Fluorescent:
                 err = sensor_write_table(info->i2c_client, sensor_WhiteB_TungstenLamp1);
			info->current_wb =YUV_Whitebalance_Fluorescent; 
                     break;
		  case YUV_Whitebalance_CloudyDaylight:
                 err = sensor_write_table(info->i2c_client, sensor_WhiteB_Cloudy);
			info->current_wb =YUV_Whitebalance_CloudyDaylight; 
                     break;
                default:
                     break;
            }
            if (err)
    	        return err;

            return 0;
        }
	 case GC0308_IOCTL_SET_YUV_EXPOSURE:
        {
	      u8 yuvexposure;
        	if (copy_from_user(&yuvexposure,
        			   (const void __user *)arg,
        			   sizeof(yuvexposure))) {
        		return -EFAULT;
        	}

            switch(yuvexposure)
            {
                case YUV_YUVExposure_Positive2:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure4);
			info->current_exposure=YUV_YUVExposure_Positive2;
                     break;
                case YUV_YUVExposure_Positive1:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure3);
		       info->current_exposure=YUV_YUVExposure_Positive1;
                     break;
                case YUV_YUVExposure_Number0:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure2);
			info->current_exposure=YUV_YUVExposure_Number0;
                     break;
                case YUV_YUVExposure_Negative1:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure1);
			info->current_exposure=YUV_YUVExposure_Negative1;
                     break;
		  case YUV_YUVExposure_Negative2:
                 err = sensor_write_table(info->i2c_client, sensor_Exposure0);
			info->current_exposure=YUV_YUVExposure_Negative2;
                     break;
                default:
                     break;
            }
            if (err)
    	        return err;
            return 0;
        }
	 case GC0308_IOCTL_SET_SCENE_MODE:
        {
              u8 scene_mode;
        	if (copy_from_user(&scene_mode,
        			   (const void __user *)arg,
        			   sizeof(scene_mode))) {
        		return -EFAULT;
        	}

            switch(scene_mode)
            {
			 case YUV_YUVSCENE_Auto:
	                 err = sensor_write_table(info->i2c_client, sensor_SceneAuto);
	                 	break;
	                case YUV_YUVSCENE_Night:
	             		err = sensor_write_table(info->i2c_client, sensor_SceneNight);
	                 	break;
			  default:
                    		 break;
		}
              if (err)
    	        return err;

            return 0;
        }
	 case GC0308_IOCTL_GET_STATUS:
	{
		struct gc0308_status dev_status;
		if (copy_from_user(&dev_status,
				   (const void __user *)arg,
				   sizeof(struct gc0308_status))) {
			return -EFAULT;
		}

		err = gc0328_get_status(info, &dev_status);
		if (err)
			return err;
		if (copy_to_user((void __user *)arg, &dev_status,
				 sizeof(struct gc0308_status))) {
			return -EFAULT;
		}
		return 0;
	}
    	default:
    		return -EINVAL;
	}
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	pr_err("gc0328 %s\n",__func__);
	file->private_data = gc0328_sensor_info;
	if (gc0328_sensor_info->pdata && gc0328_sensor_info->pdata->power_on)
		gc0328_sensor_info->pdata->power_on();
	return 0;
}

static int sensor_release(struct inode *inode, struct file *file)
{
	pr_err("gc0328 %s\n",__func__);
	if (gc0328_sensor_info->pdata && gc0328_sensor_info->pdata->power_off)
		gc0328_sensor_info->pdata->power_off();
	file->private_data = NULL;
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.unlocked_ioctl = sensor_ioctl,
	.release = sensor_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = SENSOR_NAME,
	.fops = &sensor_fileops,
};

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err;
	pr_info("gc0328 %s\n",__func__);

	gc0328_sensor_info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (!gc0328_sensor_info) {
		pr_err("gc0328 : Unable to allocate memory!\n");
		return -ENOMEM;
	}

	err = misc_register(&sensor_device);
	if (err) {
		pr_err("gc0328 : Unable to register misc device!\n");
		goto EXIT;
	}
	gc0328_sensor_info->pdata = client->dev.platform_data;
	gc0328_sensor_info->i2c_client = client;
	
	i2c_set_clientdata(client, gc0328_sensor_info);
	pr_info("gc0328 %s register successfully!\n",__func__);
	return 0;
EXIT:
	kfree(gc0328_sensor_info);
	return err;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_info *info;

	pr_info("gc0328 %s\n",__func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
    { "gc0308", 0 },
	{ "gc0328", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = "gc0308",
		.owner = THIS_MODULE,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static int __init sensor_init(void)
{
	pr_info("gc0328 %s\n",__func__);
	return i2c_add_driver(&sensor_i2c_driver);
}

static void __exit sensor_exit(void)
{
	pr_info("gc0328 %s\n",__func__);
	i2c_del_driver(&sensor_i2c_driver);
}

module_init(sensor_init);
module_exit(sensor_exit);

