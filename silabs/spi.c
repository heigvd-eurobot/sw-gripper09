/*============================================================\

   Project: Robo_09  -  Gestion Gripper

   Guillaume Galeazzi
   16.02.2009
   Heig-vd/robotique

    spi.c   

 ============================================================

   But:  Ce programme contient les routines bas niveau de communications SPI
   
\============================================================*/
#ifndef __C300__ 
#define __C300__ 
	#include "C8051F330.h"
	#include "types.h" 
#endif
#include "gripper_io.h"
#include "gripper_config.h"
#include  "gripper.h"

// global variable extern
extern GRIPPER_POS pos_actu;

// global interne
uint8    spi_index=0;
uint8    spi_buff_bras[16];
uint8    spi_buff_bilt[16];
bit      spi_state;
bit      spi_sel_DSP;

uint8    recept_MOT_BG;
uint8    recept_MOT_LT;
uint8    recept_MOT_BD;
uint8    recept_MOT_BI;


void Spi_send(uint8 motteur,uint16 angle,uint16 vitesse,uint16 couple)
{
   uint8 i=0;        // case a remplire du tableau
   uint8 *spi_buff;  // pointe vers le bon tableau
   
   while(spi_state!=SPI_FREE);   // le dernier byte est dans le buffer
   while(SPI0CFG&0x80);          // il a été envoyé
   // enlève les cs
   NSS1=1;
   NSS2=1;
   
   // si c'est au DSP qui gère les bras à qui on veu parler
   if ((motteur==MOT_GAUCHE)||(motteur==MOT_DROITE))
   {
      spi_sel_DSP=SPI_DSP_BRAS;
      NSS2=0;
      if(motteur==MOT2BRAS)   // si c'est le 2e motteur gérer par le bras, décale de 3 case
         i=6;
      spi_buff=spi_buff_bras;         
   }
   else
   {
      spi_sel_DSP=SPI_DSP_BILT;
      NSS1=0;
      if(motteur==MOT2BILT)
         i=6;      
      spi_buff=spi_buff_bilt;
   }
   
   spi_buff[i]=(angle>>8)&0xFF;
   i++;
   spi_buff[i]=angle&0xFF;
   i++;
   spi_buff[i]=(vitesse>>8)&0xFF;
   i++;
   spi_buff[i]=vitesse&0xFF;
   i++;
   spi_buff[i]=(couple>>8)&0xFF;
   i++;
   spi_buff[i]=couple&0xFF;
   
   // set et clear config = 0   
   spi_buff[12]=0x00;
   spi_buff[13]=0x00;   
   spi_buff[14]=0x00;   
   spi_buff[15]=0x00;        
   
   spi_index=0;
   SPIF = 1;      // set the irq flag
}

void Spi_clr_cfg(uint8 status, uint8 motteur)
{
   uint8 *spi_buff;
   uint8 i=0;
   
   while(spi_state!=SPI_FREE);    // le dernier byte est dans le buffer
   while(SPIBSY);               // il a été envoyé
   NSS1=1;
   NSS2=1;
   i=15;
   if ((motteur==MOT_GAUCHE)||(motteur==MOT_DROITE))
   {
      spi_sel_DSP=SPI_DSP_BRAS;
      NSS2=0;
      if(motteur==MOT2BRAS)
         i--;
      spi_buff=spi_buff_bras;         
   }
   else
   {
      spi_sel_DSP=SPI_DSP_BILT;
      NSS1=0;
      if(motteur==MOT2BILT)
         i--;
      spi_buff=spi_buff_bilt;
   }
   spi_buff[i]=status;

   spi_index=0;
   SPIF = 1;      // set the irq flag
}

void Spi_set_cfg(uint8 status, uint8 motteur)
{
   uint8 *spi_buff;
   uint8 i=0;
   
   while(spi_state!=SPI_FREE);    // le dernier byte est dans le buffer
   while(SPIBSY);               // il a été envoyé
   NSS1=1;
   NSS2=1;
   i=13;
   if ((motteur==MOT_GAUCHE)||(motteur==MOT_DROITE))
   {
      spi_sel_DSP=SPI_DSP_BRAS;
      NSS2=0;
      if(motteur==MOT2BRAS)
         i--;
      spi_buff=spi_buff_bras;         
   }
   else
   {
      spi_sel_DSP=SPI_DSP_BILT;
      NSS1=0;
      if(motteur==MOT2BILT)
         i--;
      spi_buff=spi_buff_bilt;
   }
   spi_buff[i]=status;

   spi_index=0;
   SPIF = 1;      // set the irq flag
}

void Spi_update()
{
   Spi_set_cfg(0, MOT_GAUCHE);
   Spi_set_cfg(0, MOT_LINTEAU);
 // ne modifie pas les registre mais effectue une lecture des 2 dsp
 }

void Wait_for_a_while()
{
	unsigned int i; 
	for(i=0; i<0xFFF; i++);
}

void Wait_for_a_long_while()
{
	unsigned int i,j; 
   for(j=0; j<0xFFF; j++)
      for(i=0; i<0xFFF; i++);
}

void Init_SPI()
{
   unsigned char i;
   spi_state=SPI_FREE;
   spi_index=0;
   recept_MOT_BG=0;
   recept_MOT_LT=0;
   recept_MOT_BD=0;
   recept_MOT_BI=0;

   // Fait un reset des DSP
   #if !DEBUG
      NRESET=0;
      Wait_for_a_while();
      NRESET=1;
      
      // Attend qu'il se réveille.
      Wait_for_a_long_while();
   #endif
      
   
   for(i=0;i<16;i++)    // clear spi registry 
   {
      spi_buff_bilt[i]=0;
      spi_buff_bras[i]=0;
   }  
	//wait for both DSP ready
	do {
         Spi_send(MOT_GAUCHE,0x55AA,0xAFFA,0xF0F0);
   		Spi_send(MOT_LINTEAU,0x55AA,0xAFFA,0xF0F0);
         Wait_for_a_while();
		while(spi_state!=SPI_FREE);      // le dernier byte est dans le buffer
		while(SPIBSY);                   // il a été envoyé

    } while((pos_actu.recept_MOT_BD != 0x70) && (pos_actu.recept_MOT_BG != 0xC5) && 
			(pos_actu.recept_MOT_BI != 0x70) && (pos_actu.recept_MOT_LT != 0xC5));

   // clear all registre
   Spi_send(MOT_GAUCHE,0x0000,0x0000,0x0000);
   Spi_send(MOT_DROITE,0x0000,0x0000,0x0000);
   Spi_send(MOT_LINTEAU,0x0000,0x0000,0x0000);
   Spi_send(MOT_BARILLET,0x0000,0x0000,0x0000);

}

