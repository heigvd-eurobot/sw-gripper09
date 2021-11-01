/*============================================================\

   Project: Robo_09  -  Gestion Gripper

   Guillaume Galeazzi
   16.02.2009
   Heig-vd/robotique

   gripper_io.h   

 ============================================================

   But:  Répertorier les IOs et les constantes nécessaire dans
         la partie gripper

\============================================================*/

#ifndef GRIPPER_IO_HEADER

#ifndef __C300__ 
#define __C300__ 
	#include "C8051F330.h"
	#include "types.h" 
#endif

#define GRIPPER_IO_HEADER  1
   

#define	KEIL_COMPILER 0

#if KEIL_COMPILER
	sbit vang=P0^7;   
	sbit vand=P1^0;  
	sbit pav=P1^1;
	sbit capg=P1^5;
	sbit capd=P1^4;
	sbit prsg=P1^3;       
	sbit prsd=P1^2;
	sbit capav=P1^7;
	sbit SDA=P0^5;
	sbit SCL=P0^6;
	sbit batterie=P0^4;
	// to DSP
	sbit NSS1=P1^6;
	sbit NSS2=P0^3;
	sbit SCK=P0^0;
	sbit MOSI=P0^2;
	sbit MISO=P0^1;
	sbit NRESET=P2^1;
#else
	sbit at 0x87 vang;   
	sbit at 0x90 vand;  
	sbit at 0x91 pav;
	sbit at 0x95 capg;
	sbit at 0x94 capd;
	sbit at 0x93 prsg;       
	sbit at 0x92 prsd;
	sbit at 0x97 capav;
	sbit at 0x85 SDA;
	sbit at 0x86 SCL;
	sbit at 0x84 batterie;
	// to DSP
	sbit at 0x96 NSS1;
	sbit at 0x83 NSS2;
	sbit at 0x80 SCK;
	sbit at 0x82 MOSI;
	sbit at 0x81 MISO;
	sbit at 0xA0 NRESET;
#endif

// adresse des ports du F330
// at 0x80 P0;   
// at 0x90 P1;   
// at 0xA0 P2;   

// P0.4 entrée diviseur batterie.

// to pompe a vide   

#define PAV_ON    1
#define PAV_OFF   0

#define VAN_OPEN  1
#define VAN_CLOSE 0
         // open= le vide est fait...
// to capteurs


#define CAPT_PAL    0
#define CAPT_NO_PAL  1

#define  CAPT_PRES   1
#define  CAPT_NO_PRES   0


#define CAPT_FULL    1
#define CAPT_FREE    0


#endif

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
