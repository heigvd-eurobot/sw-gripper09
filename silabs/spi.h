/*============================================================\

   Project: Robo_09  -  Gestion Gripper

   Guillaume Galeazzi
   16.02.2009
   Heig-vd/robotique

    spi.h   

 ============================================================

   But:  
   
   Ce programme contient les routines bas niveau de 
   communications SPI
   
   Comment:
   
   vérifier que les retours du DSP vont dans les bonnes variables recept_MOT_**
   
\============================================================*/
#ifndef SPI_HEADER

#define SPI_HEADER  1

#ifndef __C300__ 
#define __C300__ 
	#include "C8051F330.h"
	#include "types.h" 
#endif

#include "types.h"


void Init_SPI();
void Spi_send(uint8 motteur,uint16 angle,uint16 vitesse,uint16 couple);
void Spi_clr_cfg(uint8 status, uint8 motteur);
void Spi_set_cfg(uint8 status, uint8 motteur);
void Spi_update();
// met a jour les valeurs de recept_MOT_G et recept_MOT_D

#endif

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
