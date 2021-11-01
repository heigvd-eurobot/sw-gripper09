#ifndef __C300__
#define __C300__
#include "C8051F330.h"
#include "types.h"
#endif
#include "gripper_io.h"
#include "gripper_config.h"
#include "gripper.h"
#include "spi.h"

// global variable intern
GRIPPER_POS pos_actu;

bit synchro;
bit brasg_rdy;
bit brasd_rdy;
bit lock_bi_g;
bit lock_bi_d;

void Homing_Mott()
{
   uint16 i;
   uint8 homing;
   Spi_update();
   while (!STATU_FREE(pos_actu.recept_MOT_BG & pos_actu.recept_MOT_BD & pos_actu.recept_MOT_BI & pos_actu.recept_MOT_LT))
   // attend que les dsp soit libre
   {
      for (i = 1; i != 0; i++)
         ;
      Spi_update();
   }

   // envoie les homing des bras + barilet
   Spi_set_cfg(MOT_START | MOT_HOME, MOT_BARILLET);
   Spi_set_cfg(MOT_START | MOT_HOME, MOT_GAUCHE);
   Spi_set_cfg(MOT_START | MOT_HOME, MOT_DROITE);

   homing = 0;
   while (!homing)
   {
      for (i = 1; i != 0; i++)
         ;
      Spi_update();
      if (STATU_HOME_DN(pos_actu.recept_MOT_BG & pos_actu.recept_MOT_BD & pos_actu.recept_MOT_BI))
      {
         homing = 1;
      }
      if (STATU_ERR(pos_actu.recept_MOT_BG | pos_actu.recept_MOT_BD))
      {
         homing = 2;
      }
   }
   if (homing == 1) // le homing des bras et du barilet a r�ussi.
   {
      Spi_set_cfg(MOT_START | MOT_HOME, MOT_LINTEAU); // homing linteau
      while (STATU_HOME_DN(pos_actu.recept_MOT_LT))
      {
         for (i = 1; i != 0; i++)
            ;
         Spi_update();
      }
   }
   else
   {
      Spi_clr_cfg(SPI_CLR_ERR, MOT_GAUCHE);
      Spi_clr_cfg(SPI_CLR_ERR, MOT_DROITE);

      Spi_set_cfg(MOT_START | MOT_HOME, MOT_LINTEAU); // homing linteau
      while (STATU_HOME_DN(pos_actu.recept_MOT_LT))
      {
         for (i = 1; i != 0; i++)
            ;
         Spi_update();
      }
      Spi_set_cfg(MOT_START | MOT_HOME, MOT_GAUCHE);
      Spi_set_cfg(MOT_START | MOT_HOME, MOT_DROITE);
      while (STATU_HOME_DN(pos_actu.recept_MOT_BG & pos_actu.recept_MOT_BD))
      {
         for (i = 1; i != 0; i++)
            ;
         Spi_update();
      }
   }
}

void Init_Gripper()
{

   // initialise les variables
   pos_actu.bras_gauche = X_BRAS_HOME;
   pos_actu.bras_droite = X_BRAS_HOME;
   pos_actu.barillet = BARILLET_HOME;
   pos_actu.linteau = LINTEAU_HOME;

   pos_actu.h_tower_g = 0;
   pos_actu.h_tower_d = 0;
   pos_actu.nb_pallet = 0;
   pos_actu.nb_linteau = 3;
   lock_bi_d = 0;
   lock_bi_g = 0;
   pos_actu.manual_mode = 0;

   synchro = BRAS_NSYNC;
   brasg_rdy = BRAS_NRDY;
   brasd_rdy = BRAS_NRDY;

   // initialise les machines d'�tats
   pos_actu.state_bg = SBG_REPOT;
   pos_actu.state_bd = SBD_REPOT;
   pos_actu.state_lt = SLT_REPOT;
   pos_actu.state_bi = SBI_REPOT;

   Homing_Mott();

   // met les moteurs dans la position de repos
   Spi_send(MOT_DROITE, X_BRAS_POSE_VER, V_SLOW, C_SLOW);
   Spi_send(MOT_GAUCHE, X_BRAS_POSE_VER, V_SLOW, C_SLOW);
   Spi_send(MOT_BARILLET, BARILLET_OPEN1, V_SLOW, C_SLOW);
   Spi_send(MOT_LINTEAU, LINTEAU_IN, V_SLOW, C_SLOW);

   // initialise le tableau
   pos_actu.pos_pallet[0] = B_NO_PAL;
   pos_actu.pos_pallet[1] = B_NO_PAL;
   pos_actu.pos_pallet[2] = B_NO_PAL;
   pos_actu.pos_pallet[3] = B_NO_PAL;
}

void State_machine_bg()
{
   static bit state_changed = 1;
   static uint8 local_static_tmp;
   static uint8 memo_state;

   if (state_changed || (memo_state != pos_actu.state_bg)) // test si une irq a chang� l'�tat
      state_changed = 1;
   else
      state_changed = 0;

   memo_state = pos_actu.state_bg;
   switch (pos_actu.state_bg)
   {
   case SBG_REPOT:
      //vand=0;
      if (state_changed)
      {
         // envoye l'ordre de monter le bras
         Spi_send(MOT_GAUCHE, X_BRAS_POSE_VER, V_MED, C_SLOW);
      }
      break;

   case SBG_SCAN_AV:
      if (state_changed)
      {
         local_static_tmp = NB_MAX_PAL_TOWER + 1;
      }
      if (STATU_POS_ATT(pos_actu.recept_MOT_BG))
      {
         if (local_static_tmp != 0)
            local_static_tmp--;
         else
            pos_actu.state_bg = SBG_SCAN_ER;
         Spi_send(MOT_GAUCHE, X_BRAS_POSE_SOL + local_static_tmp * X_BRAS_TOWER_1PAL, V_MED, C_SLOW);
      }
      else if (STATU_OVR_CPL(pos_actu.recept_MOT_BG))
      {
         pos_actu.h_tower_g = local_static_tmp;
         state_changed = 1;
         pos_actu.state_bg = SBG_REPOT;
      }
      break;

   case SBG_PRISE_PAL_1:
      if (state_changed)
      {
         Spi_send(MOT_GAUCHE, X_BRAS_PRISE_BARILLET_UP, V_MED, C_MED);
         lock_bi_g = 1;
         local_static_tmp = 4;
         if (!lock_bi_d)
            pos_actu.state_bi = SBI_PRISE_G; // met l'�tat du barilet en prise barilet
      }
      if ((pos_actu.state_bi == SBI_PRISE) && (STATU_POS_ATT(pos_actu.recept_MOT_BG)) && (STATU_POS_ATT(pos_actu.recept_MOT_BI))) // quand le barilet est en place
      {
         if (capg == CAPT_PAL)
            pos_actu.state_bg = SBG_PRISE_PAL_2;
         else
         {
            if (!lock_bi_d) // si il n'y a pas de verrou
            {
               pos_actu.state_bi = SBI_ROT_1POS;
               local_static_tmp--;
            }
            if (!local_static_tmp)
            {
               lock_bi_g = 0;
               pos_actu.state_bg = SBG_ERR_NO_PAL;
            }
         }
      }
      break;

   case SBG_PRISE_PAL_2:
      if (state_changed)
      {
         Spi_send(MOT_GAUCHE, X_BRAS_PRISE_BARILLET, V_MED, C_SLOW);
         local_static_tmp = 100;
      }
      if (STATU_OVR_CPL(pos_actu.recept_MOT_BG))
      {
         vang = VAN_OPEN;
         if (prsg == CAPT_PRES) // si il y a le vide
         {
            uint8 i;
            i = pos_actu.pos_barillet >> 1;
            i = (i + 2) % 4;
            // ici i repr�sente l'index du pallet sous le bras gauche
            pos_actu.pos_pallet[i % 4] = B_NO_PAL; // on el�ve le palet du tableau
            pos_actu.h_tower_g++;
            pos_actu.state_bg = SBG_PRISE_PAL_3;
            Spi_send(MOT_GAUCHE, X_BRAS_POSE_VER, V_SLOW, C_HARD);
         }
         else
         {
            local_static_tmp--;
            if (!local_static_tmp)
            {
               lock_bi_g = 0;
               pos_actu.state_bg = SBG_ERR_NO_PAL;
            }
         }
      }
      break;

   case SBG_PRISE_PAL_3:
      if (lock_bi_g && STATU_POS_ATT(pos_actu.recept_MOT_BG))
      {
         lock_bi_g = 0;
         Spi_send(MOT_GAUCHE, X_BRAS_POSE_SOL + pos_actu.h_tower_g * X_BRAS_TOWER_1PAL, V_SLOW, C_HARD);
      }
      if (STATU_POS_ATT(pos_actu.recept_MOT_BG) || STATU_OVR_CPL(pos_actu.recept_MOT_BG))
      {
         vang = VAN_CLOSE;
         pos_actu.state_bg = SBG_REPOT;
      }
      break;

   case SBG_POSE_LT:
      if (STATU_POS_ATT(pos_actu.recept_MOT_BG))
      {
         if (state_changed)
         {
            pos_actu.state_lt = SLT_OUT; // sort le linteau
                                         // il me changera d'�tat apr�s
            Spi_send(MOT_GAUCHE, X_BRAS_POSE_LT_UP_UP, V_MED, C_MED);
         }
         if (STATU_POS_ATT(pos_actu.recept_MOT_LT)) // quand le linteau est sorti
         {
            if (capg == CAPT_PAL)
            {
               brasg_rdy == BRAS_NRDY;
               pos_actu.state_bg = SBG_POSE_LT_2;
            }
            else
            {
               Spi_send(MOT_GAUCHE, X_BRAS_POSE_LT_UP, V_MED, C_MED);
            }
         }
      }
      break;

   case SBG_POSE_LT_2:
      if (state_changed)
      {
         Spi_send(MOT_GAUCHE, X_BRAS_POSE_LT, V_SLOW, C_SLOW);
         // checker ovr_cpl ou attendre fin a voir
      }
      if (STATU_OVR_CPL(pos_actu.recept_MOT_BG))
      {
         if ((brasd_rdy == BRAS_RDY) && (pos_actu.state_bd == SBD_POSE_LT_2)) // si l'autre bras est d�j� pr�s
         {
            // d�sactive le mode de d�marrage automatique des commandes
            Spi_clr_cfg(MOT_START, MOT_GAUCHE);

            vang = VAN_OPEN;
            vand = VAN_OPEN;

            // envoie d�j� l'ordre d'apr�s
            Spi_send(MOT_GAUCHE, X_BRAS_POSE_SOL + local_static_tmp * X_BRAS_TOWER_1PAL, V_SLOW, C_SLOW);
            Spi_send(MOT_DROITE, X_BRAS_POSE_SOL + local_static_tmp * X_BRAS_TOWER_1PAL, V_SLOW, C_SLOW);

            if ((prsg == CAPT_PRES) && (prsd == CAPT_PRES))
            {
               pos_actu.nb_linteau--;
               Spi_set_cfg(MOT_START, MOT_GAUCHE); // d�marre le mouvement
               Spi_set_cfg(MOT_START, MOT_DROITE); // d�marre le mouvement
               // checker ovr_cpl ou attendre fin a voir

               pos_actu.state_bg = SBG_POSE_LT_3;
               pos_actu.state_bd = SBD_POSE_LT_3;
               brasg_rdy = BRAS_NRDY;
               brasd_rdy = BRAS_NRDY;
            }
         }
         else
         {
            brasg_rdy = BRAS_RDY;
         }
      }
      break;

   case SBG_POSE_LT_3:
      if (STATU_OVR_CPL(pos_actu.recept_MOT_BG) || STATU_POS_ATT(pos_actu.recept_MOT_BG))
      {
         if (brasd_rdy == BRAS_RDY) // si l'autre bras est d�j� pr�s
         {
            vang = VAN_CLOSE;
            vand = VAN_CLOSE;

            pos_actu.state_bg = SBG_REPOT;
            pos_actu.state_bd = SBD_REPOT;

            brasd_rdy = BRAS_NRDY;
            brasg_rdy = BRAS_NRDY;

            pos_actu.h_tower_d++;
            pos_actu.h_tower_g++;
         }
         else
         {
            brasg_rdy = BRAS_RDY;
         }
      }
      break;
   }

   if (memo_state != pos_actu.state_bg) // test si la ms a chang� d'�tat
      state_changed = 1;
   else
      state_changed = 0;

   memo_state = pos_actu.state_bg;

   // t>TIME_OUT signaler erreur!
}

void State_machine_bd()
{
   static bit state_changed = 1;
   static uint8 local_static_tmp;
   static uint8 memo_state;

   if (state_changed || (memo_state != pos_actu.state_bd)) // test si une irq a chang� l'�tat
      state_changed = 1;
   else
      state_changed = 0;

   memo_state = pos_actu.state_bd;
   switch (pos_actu.state_bd)
   {
   case SBD_REPOT:
      if (state_changed)
      {
         // envoye l'ordre de monter le bras
         Spi_send(MOT_GAUCHE, X_BRAS_POSE_VER, V_MED, C_SLOW);
      }
      break;

   case SBD_SCAN_AV:
      if (state_changed)
      {
         local_static_tmp = NB_MAX_PAL_TOWER;
      }
      if (STATU_POS_ATT(pos_actu.recept_MOT_BD))
      {
         if (local_static_tmp != 0)
            local_static_tmp--;
         else
            pos_actu.state_bd = SBD_SCAN_ER;
         Spi_send(MOT_DROITE, X_BRAS_POSE_SOL + local_static_tmp * X_BRAS_TOWER_1PAL, V_MED, C_SLOW);
      }
      else if (STATU_OVR_CPL(pos_actu.recept_MOT_BG))
      {
         pos_actu.h_tower_d = local_static_tmp;
         state_changed = 1;
         pos_actu.state_bd = SBD_REPOT;
      }
      break;

   case SBD_PRISE_PAL_1:
      if (state_changed)
      {
         Spi_send(MOT_DROITE, X_BRAS_PRISE_BARILLET_UP, V_MED, C_MED);
         lock_bi_d = 1;
         local_static_tmp = 4;
         if (!lock_bi_g)
            pos_actu.state_bi = SBI_PRISE_D;
      }
      if ((pos_actu.state_bi == SBI_PRISE) && (STATU_POS_ATT(pos_actu.recept_MOT_BD)) && (STATU_POS_ATT(pos_actu.recept_MOT_BI)))
      {
         if (capd == CAPT_PAL)
            pos_actu.state_bd = SBD_PRISE_PAL_2;
         else
         {
            if (!lock_bi_g)
            {
               pos_actu.state_bi = SBI_PRISE_D;
               local_static_tmp--;
            }
            if (!local_static_tmp)
            {
               lock_bi_d = 0;
               pos_actu.state_bd = SBD_ERR_NO_PAL;
            }
         }
      }
      break;

   case SBD_PRISE_PAL_2:
      if (state_changed)
      {
         Spi_send(MOT_DROITE, X_BRAS_PRISE_BARILLET, V_MED, C_SLOW);
         local_static_tmp = 100;
      }
      if (STATU_OVR_CPL(pos_actu.recept_MOT_BD))
      {
         vand = VAN_OPEN;
         if (prsd == CAPT_PRES) // si il y a le vide
         {
            uint8 i;
            i = pos_actu.pos_barillet >> 1;
            i = (i + 3) % 4;
            // ici i repr�sente l'index du pallet sous le bras gauche
            pos_actu.pos_pallet[i % 4] = B_NO_PAL; // on el�ve le palet du tableau
            pos_actu.h_tower_d++;
            pos_actu.state_bd = SBD_PRISE_PAL_3;
            Spi_send(MOT_DROITE, X_BRAS_POSE_VER, V_SLOW, C_HARD);
         }
         else
         {
            local_static_tmp--;
            if (!local_static_tmp)
            {
               lock_bi_g = 0;
               pos_actu.state_bg = SBG_ERR_NO_PAL;
            }
         }
      }
      break;

   case SBD_PRISE_PAL_3:
      if (lock_bi_g && STATU_POS_ATT(pos_actu.recept_MOT_BD))
      {
         lock_bi_d = 0;
         Spi_send(MOT_DROITE, X_BRAS_POSE_SOL + pos_actu.h_tower_d * X_BRAS_TOWER_1PAL, V_SLOW, C_HARD);
      }
      if (STATU_POS_ATT(pos_actu.recept_MOT_BD) || STATU_OVR_CPL(pos_actu.recept_MOT_BD))
      {
         vand = VAN_CLOSE;
         pos_actu.state_bd = SBD_REPOT;
      }
      break;

   case SBD_POSE_LT:
      if (STATU_POS_ATT(pos_actu.recept_MOT_BD))
      {
         if (state_changed)
         {
            pos_actu.state_lt = SLT_OUT; // sort le linteau
                                         // il me changera d'�tat apr�s
            Spi_send(MOT_DROITE, X_BRAS_POSE_LT_UP_UP, V_MED, C_MED);
         }
         if (STATU_POS_ATT(pos_actu.recept_MOT_LT)) // quand le linteau est sorti
         {
            if (capd == CAPT_PAL)
            {
               brasd_rdy == BRAS_NRDY;
               pos_actu.state_bd = SBD_POSE_LT_2;
            }
            else
            {
               Spi_send(MOT_GAUCHE, X_BRAS_POSE_LT_UP, V_MED, C_MED);
            }
         }
      }
      break;

   case SBD_POSE_LT_2:
      if (state_changed)
      {
         Spi_send(MOT_DROITE, X_BRAS_POSE_LT, V_SLOW, C_SLOW);
         // checker ovr_cpl ou attendre fin a voir
      }
      if (STATU_OVR_CPL(pos_actu.recept_MOT_BG))
      {
         if ((brasg_rdy == BRAS_RDY) && (pos_actu.state_bg == SBG_POSE_LT_2)) // si l'autre bras est d�j� pr�s
         {
            // d�sactive le mode de d�marrage automatique des commandes
            Spi_clr_cfg(MOT_START, MOT_DROITE);

            vang = VAN_OPEN;
            vand = VAN_OPEN;

            // envoie d�j� l'ordre d'apr�s
            Spi_send(MOT_DROITE, X_BRAS_POSE_SOL + local_static_tmp * X_BRAS_TOWER_1PAL, V_SLOW, C_SLOW);
            Spi_send(MOT_GAUCHE, X_BRAS_POSE_SOL + local_static_tmp * X_BRAS_TOWER_1PAL, V_SLOW, C_SLOW);

            if ((prsg == CAPT_PRES) && (prsd == CAPT_PRES))
            {
               pos_actu.nb_linteau--;
               Spi_set_cfg(MOT_START, MOT_GAUCHE); // d�marre le mouvement
               Spi_set_cfg(MOT_START, MOT_DROITE); // d�marre le mouvement
               // checker ovr_cpl ou attendre fin a voir

               pos_actu.state_bg = SBG_POSE_LT_3;
               pos_actu.state_bd = SBD_POSE_LT_3;
               brasg_rdy = BRAS_NRDY;
               brasd_rdy = BRAS_NRDY;
            }
         }
         else
         {
            brasd_rdy = BRAS_RDY;
         }
      }
      break;

   case SBD_POSE_LT_3:
      if (STATU_OVR_CPL(pos_actu.recept_MOT_BG) || STATU_POS_ATT(pos_actu.recept_MOT_BG))
      // le couple a �t� d�pass�, on est en contacte avec le linteau
      {
         if (brasg_rdy == BRAS_RDY) // si l'autre bras est d�j� pr�s
         {
            vang = VAN_CLOSE;
            vand = VAN_CLOSE;

            pos_actu.state_bg = SBG_REPOT;
            pos_actu.state_bd = SBD_REPOT;

            brasd_rdy = BRAS_NRDY;
            brasg_rdy = BRAS_NRDY;

            pos_actu.h_tower_d++;
            pos_actu.h_tower_g++;
         }
         else
         {
            brasd_rdy = BRAS_RDY;
         }
      }
      break;
   }
   if (memo_state != pos_actu.state_bd) // test si la ms a chang� d'�tat
      state_changed = 1;
   else
      state_changed = 0;

   memo_state = pos_actu.state_bd;

   // t>TIME_OUT signaler erreur!
}
void State_machine_lt()
{
   // l'�tat est chang� a l'ext�rieur de cet machine...
   static bit linteau_in = 0;

   if (STATU_POS_ATT(pos_actu.recept_MOT_LT)) // si la position demand� est atteinte
   {
      switch (pos_actu.state_lt)
      {
      case SLT_REPOT: // linteau rentrer
         if (linteau_in == 0)
         {
            // envoye l'ordre de rentrer le linteau
            Spi_send(MOT_LINTEAU, LINTEAU_IN, V_MED, C_MED);
            linteau_in == 1;
         }
         break;

      case SLT_OUT: // linteau sorti
         if (linteau_in == 1)
         {
            // envoye l'ordre de sortir le linteau
            Spi_send(MOT_LINTEAU, LINTEAU_OUT, V_MED, C_MED);
            linteau_in = 0;
         }
         break;
      }
   }
   else
   {
      // t>TIME_OUT signaler erreur!
   }
}
void State_machine_bi()
{
   if (STATU_POS_ATT(pos_actu.recept_MOT_BI)) // si la position demand� est atteinte
   {
      switch (pos_actu.state_bi)
      {
      case SBI_REPOT:
         if (pos_actu.nb_pallet != 4) // si il n'est pas plein, j'attend que des pallet arrive
         {
            pos_actu.state_bi = SBI_ATT_PAL;
            // Spi_send(MOT_BARILLET,BARILLET_OPEN1,V_MED,C_MED);
            // pos_actu.pos_barillet=0;
         }
         else // si il est plein je me pr�part pour les bras
         {
            pos_actu.state_bi = SBI_PLEIN;
            if (!(pos_actu.pos_barillet & 0x01)) // si il est ouvert
            {
               pos_actu.pos_barillet = (pos_actu.pos_barillet + 1) % 8;
               Spi_send(MOT_BARILLET, BARILLET_OPEN1 + (pos_actu.pos_barillet * BARILLET_DEMI_POS), V_MED, C_MED);
            }
         }

         break;

      case SBI_ATT_PAL: // attend qu'un pallet arrive dans le barilet
         if (pos_actu.nb_pallet == 4)
         {
            pos_actu.state_bi == SBI_REPOT;
         }
         else if (pos_actu.pos_barillet & 0x01) // le barrilet est ferm�
         {
            pos_actu.pos_barillet = (pos_actu.pos_barillet + 1) % 8;
            Spi_send(MOT_BARILLET, BARILLET_OPEN1 + (pos_actu.pos_barillet * BARILLET_DEMI_POS), V_MED, C_MED);
         }
         else if (pos_actu.pos_pallet[pos_actu.pos_barillet >> 1] != B_NO_PAL) // la place n'�tait pas vide
         {
            pos_actu.pos_barillet = (pos_actu.pos_barillet + 2) % 8;
            Spi_send(MOT_BARILLET, BARILLET_OPEN1 + (pos_actu.pos_barillet * BARILLET_DEMI_POS), V_MED, C_MED);
         }
         else if ((capav == CAPT_FULL)) // un pallet est entr�e
         {
            pos_actu.nb_pallet++;
            pos_actu.pos_barillet = (pos_actu.pos_barillet + 2) % 8;
            Spi_send(MOT_BARILLET, BARILLET_OPEN1 + (pos_actu.pos_barillet * BARILLET_DEMI_POS), V_MED, C_MED);
         }
         break;

      case SBI_PLEIN: // le barilet est plein
          // attend qu'on passe a demande pallet -> demand� par state_machine_bx
         break;

      case SBI_PRISE_D: // met un pallet devant le bras droite
      {
         uint8 i, nb_pos;
         i = pos_actu.pos_barillet >> 1;
         i = (i + 3) % 4;
         // ici i repr�sente l'index du pallet sous le bras gauche
         for (nb_pos = 0; (pos_actu.pos_pallet[(i + nb_pos) % 4] == B_NO_PAL) && (nb_pos < 4); nb_pos++)
         {
         }
         // ici nb_pos vaut le nombre de position qu'il faut tourner pour qu'on ai un palet en face du bras
         if (nb_pos != 4) // il y a un pallet :)
         {
            Spi_send(MOT_BARILLET, BARILLET_OPEN1 + (nb_pos * BARILLET_1POS), V_MED, C_MED);
            pos_actu.pos_barillet = (pos_actu.pos_barillet + 2 * nb_pos) % 8;
            pos_actu.state_bi = SBI_PRISE_D;
         }
         else // il n'y a pas de pallet :(
         {
            // printf("On est pas dans la merde...");
            pos_actu.state_bi = SBI_ERR_NO_PAL;
         }
      }
      break;

      case SBI_PRISE_G: // met un pallet devant le bras gauche
      {
         uint8 i, nb_pos;
         i = pos_actu.pos_barillet >> 1;
         i = (i + 2) % 4;
         // ici i repr�sente l'index du pallet sous le bras gauche
         for (nb_pos = 0; (pos_actu.pos_pallet[(i + nb_pos) % 4] == B_NO_PAL) && (nb_pos < 4); nb_pos++)
         {
         }
         // ici nb_pos vaut le nombre de position qu'il faut tourner pour qu'on ai un palet en face du bras
         if (nb_pos != 4) // il y a un pallet :)
         {
            Spi_send(MOT_BARILLET, BARILLET_OPEN1 + (nb_pos * BARILLET_1POS), V_MED, C_MED);
            pos_actu.state_bi = SBI_PRISE_G;
         }
         else // il n'y a pas de pallet :(
         {
            // printf("On est pas dans la merde...");
            pos_actu.state_bi = SBI_ERR_NO_PAL;
         }
      }
      break;

      case SBI_PRISE:                  // les bras prennent les palet dans le barilet
         if (!lock_bi_d && !lock_bi_g) // attend que les bras enl�ve les v�rous
            pos_actu.state_bi = SBI_REPOT;
         break;

      case SBI_ROT_1POS:                                             // fait tourner le barrillet d'une position
         if ((lock_bi_d && !lock_bi_g) || (!lock_bi_d && lock_bi_g)) // il faut qu'il y ai qu'un verrou
         {
            pos_actu.pos_barillet = (pos_actu.pos_barillet + 2) % 8;
            Spi_send(MOT_BARILLET, BARILLET_OPEN1 + (pos_actu.pos_barillet * BARILLET_DEMI_POS), V_MED, C_MED);
            pos_actu.state_bi = SBI_PRISE;
         }
         break;
      }
   }
   else
   {
      // t>TIME_OUT signaler erreur!
   }
}
