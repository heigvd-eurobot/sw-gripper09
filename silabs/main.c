/**
 * Project: Robo_09  -  Gestion Gripper
 * Guillaume Galeazzi
 * 16.02.2009
 * Programme principale de la carte gripper, gestion des
 * deux DSP, de la pression et des différents capteurs.
 */
#ifndef __C300__
#define __C300__
#include "C8051F330.h"
#include "types.h"
#endif

#include "initialisation.h"
#include "gripper_io.h"
#include "gripper_config.h"
#include "gripper.h"
#include "spi.h"
#include "i2c.h"

// Global variable
uint16 vbatterie = 330;  // tension de la baterie en 0.1V
typ_err erreur = E_NONE; // code d'erreur
uint16 timer_cpt;        // compteur d'irq timer0 pour candencer le main

// variables du fichier spi.c
extern uint8 spi_index;
extern uint8 spi_buff_bras[16];
extern uint8 spi_buff_bilt[16];
extern bit spi_state;
extern bit spi_sel_DSP;

// variables du fichier gripper.c
extern GRIPPER_POS pos_actu;


//#define I2C_ADDR	0x50	// TODO: YCR: C'est quoi cette connerie ici ?

void i2c_irq2(void) interrupt 7 { i2c_isr(); }
void i2c_timeout_irq2(void) interrupt 14 { timer_scl_low_timeout_isr(); }

void main()
{

   Init_Device(); // initialise les périphériques interne

   i2c_init(I2C_GRIPPER_ADD, &pos_actu, sizeof pos_actu);

#if SPI_FAST
   SPI0CKR = 0x0B; // diviseur du spi Fspi=(sysclk)/(2xS(PI0CKR+1))
#else
   SPI0CKR = 0x79; // diviseur du spi Fspi=(sysclk)/(2xS(PI0CKR+1))
#endif
   NSS1 = 1;
   NSS2 = 1;
   IRQ_ON;
   TR0 = 1; // start timer

   pav = PAV_OFF;    // désactive la pompe
   vand = VAN_CLOSE; // ferme les vannes
   vang = VAN_CLOSE;

   IRQ_ON;

   Init_SPI(); // initialise le spi

   vang = VAN_OPEN;

   Init_Gripper();

   vand = VAN_OPEN;

   timer_cpt = 0;

   while (1)
   {
      while (pos_actu.manual_mode == 0xA5)
      {
         uint16 memo_bg = X_BRAS_POSE_VER, memo_bd = X_BRAS_POSE_VER, memo_lt = LINTEAU_IN, memo_bi = BARILLET_HOME;

         // si une des positions a changé
         if (memo_bg != pos_actu.bras_gauche)
         {
            memo_bg = pos_actu.bras_gauche;
            Spi_send(MOT_GAUCHE, memo_bg, V_MED, C_MED);
         }
         if (memo_bd != pos_actu.bras_droite)
         {
            memo_bd = pos_actu.bras_droite;
            Spi_send(MOT_DROITE, memo_bd, V_MED, C_MED);
         }
         if (memo_bi != pos_actu.barillet)
         {
            memo_bi = pos_actu.barillet;
            Spi_send(MOT_BARILLET, memo_bi, V_MED, C_MED);
         }
         if (memo_lt != pos_actu.linteau)
         {
            memo_lt = pos_actu.linteau;
            Spi_send(MOT_LINTEAU, memo_lt, V_MED, C_MED);
         }
         // copie les sortiees
         vand = pos_actu.vand;
         vang = pos_actu.vang;

         // copie les entrées
         pos_actu.capg = capg;
         pos_actu.capd = capd;
         pos_actu.capav = capav;
         pos_actu.prsg = prsg;
         pos_actu.prsd = prsd;

         while (timer_cpt < 625)
            ; // attend 625*40[us] == 25 [ms]
         timer_cpt = 0;

         Spi_update();
      }
      while (pos_actu.manual_mode != 0xA5)
      {
         State_machine_bg();
         State_machine_bd();
         State_machine_lt();
         State_machine_bi();
         while (timer_cpt < 625)
            ; // nombre de fois ou l'on attend 40[us] == 25 [ms]
         timer_cpt = 0;

         Spi_update();
      }
   }
}

void Timer0_irq(void) interrupt 1
{
   static uint16 t_on = 186;
   static uint8 i = 0;

   TF0 = 0; // clear irq flag

   if (pav == PAV_ON)
   {

      TH0 = 256 - t_on;
      pav = PAV_OFF;
   }
   else
   {
      if ((vand == VAN_OPEN) || (vang == VAN_OPEN)) // si on veu la dépression quelque part
      {
         pav = PAV_ON;

#if ADJUST_PWM
         {
            i++;
            if (i == 200) // toute les 800 ms
            {
               i = 0;
               t_on = (uint16)256 * 240 / vbatterie; // une conversion de retard, mais moin de temps cpu
               AD0BUSY = 1;                          // start convertions
            }
         }
#endif
         TH0 = t_on;
      }
      else
      {
         TH0 = 0;
      }
      timer_cpt++;
   }
}

void ADC_irq(void) interrupt 10
{
   uint16 val;
   AD0INT = 0; // clear flag
   val = ADC0H << 8 | ADC0L;
   val = val * 33 / 1024; // tension sur la pin 0.4 en 0.1V
   val = val * 11;        // tension de la batterie
   vbatterie = (vbatterie + val) / 2;
}

void Spi_irq(void) interrupt 6
{
   SPIF = 0; //clear irq flag

   if (spi_state == SPI_FREE) // si l'interruptions survien et que je suis en free, c'est que le buffer a changé
   {
      if (spi_index != 15)
         spi_state = SPI_BSY; // on va envoyé
      else                    // on vien de finir une communication
      {
         NSS1 = 1; // enléve le cs
         NSS2 = 1;
      }
   }

   if (!SPIBSY && (spi_state == SPI_BSY)) // le module spi n'est pas occupé et qu'on a qqch a envoyé
   {
      // récupére les info des DSPs
      if (spi_index == 14)
      {
         if (spi_sel_DSP == SPI_DSP_BRAS)
         {
            pos_actu.recept_MOT_BG = SPI0DAT;
         }
         else
         {
            pos_actu.recept_MOT_LT = SPI0DAT;
         }
      }
      if (spi_index == 15)
      {
         if (spi_sel_DSP == SPI_DSP_BRAS)
         {
            pos_actu.recept_MOT_BD = SPI0DAT;
         }
         else
         {
            pos_actu.recept_MOT_BI = SPI0DAT;
         }
      }

      if (spi_sel_DSP == SPI_DSP_BRAS)
      {
         // envoye un élément du  buffer
         SPI0DAT = spi_buff_bras[spi_index];
      }
      else
      {
         // envoye un élément du  buffer
         SPI0DAT = spi_buff_bilt[spi_index];
      }

      // déplace l'indexe
      if (spi_index == 15)
      {
         spi_state = SPI_FREE; // on vien d'envoyé le dernier byte
      }
      else
      {
         spi_index++;
      }
   }
}
