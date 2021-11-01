/**
 * Project: Robo_09  -  Gestion Gripper
 * Guillaume Galeazzi
 * 16.02.2009
 * Ce programme contiend les routines de gestion des 2 bras, du barillets et du linteau
 */
#ifndef GRIPPER_HEADER
#define GRIPPER_HEADER 1

typedef struct
{
   uint8 broadcast;

   uint8 h_tower_g; // hauteur de la tour devant le bras en nombre de palet
   uint8 h_tower_d;

   // registre d'�tat des diff�rentes machines
   uint8 state_bg;
   uint8 state_bd;
   uint8 state_lt;
   uint8 state_bi;

   uint8 nb_linteau;

   uint8 nb_pallet; // nombre de pallet disponnible dans le barillet

   uint8 pos_pallet[4]; // position des pallet dans le barillet
   uint8 pos_barillet;  // compte les demi positions du barrilet (open1, close1, open2, close 2 ,...
                        /*
   
   [av]
     ---+   
        |
    1   |  0  |
        |     |
  +-----+-----+
  |     |      
  | 2   |  3   
        |
        +---
        
   [ar]
   
*/
   uint8 manual_mode;
   uint16 bras_gauche;
   uint16 bras_droite;
   uint16 linteau;
   uint16 barillet;
   uint8 recept_MOT_BG;
   uint8 recept_MOT_LT;
   uint8 recept_MOT_BD;
   uint8 recept_MOT_BI;

   bit vang;
   bit vand;
   bit capg;
   bit capd;
   bit prsg;
   bit prsd;
   bit capav;
} GRIPPER_POS;

#define B_NO_PAL 0
#define B_PAL_GOOD 1
#define B_PAL_BAD 2

#define STATU_ERR(a) ((a)&0x01)
#define STATU_OVR_CPL(a) ((a)&0x02)
#define STATU_POS_ATT(a) ((a)&0x04)
#define STATU_HOME_DN(a) ((a)&0x08)
#define STATU_FREE(a) ((a)&0x10)

// Gestion des erreurs
typedef uint8 typ_err;
#define E_NONE 0     // Pas d'erreur
#define E_TIME_OUT 1 // Temps pr�vu pour le mouvement d�pass�
#define E_OVR_CPL 2  // Couple d�pass�
#define E_MOVE 3     // Erreur impr�vue durant le mouvement
#define E_OBSTACLE 4 // Un obstacle est devant le bras
#define E_NO_PAL 5   // Il n'y a plus de pallet dans le robot
#define E_PARAM 6    // les param�tres pass� a la fonction est sont faux

// Gestion des �tats
#define BRAS_SYNC 0
#define BRAS_NSYNC 1

#define BRAS_NRDY 0
#define BRAS_RDY 1

//** Bras Gauche
#define SBG_REPOT 1
#define SBG_SCAN_AV 2
#define SBG_PRISE_PAL_1 4
#define SBG_PRISE_PAL_2 5
#define SBG_PRISE_PAL_3 6
#define SBG_POS_PAL_TAB 7
#define SBG_POS_PAL_TOW 8
#define SBG_POSE_LT 9
#define SBG_POSE_LT_2 10
#define SBG_POSE_LT_3 11
#define SBG_ERR_NO_PAL 52
#define SBG_SCAN_ER 53

//** Bras Droite
#define SBD_REPOT 1
#define SBD_SCAN_AV 2
#define SBD_PRISE_PAL_1 4
#define SBD_PRISE_PAL_2 5
#define SBD_PRISE_PAL_3 6
#define SBD_POS_PAL_TAB 7
#define SBD_POS_PAL_TOW 8
#define SBD_POSE_LT 9
#define SBD_POSE_LT_2 10
#define SBD_POSE_LT_3 11
#define SBD_ERR_NO_PAL 52
#define SBD_SCAN_ER 53

//** Linteau
#define SLT_REPOT 1
#define SLT_OUT 2

//** Barillet
#define SBI_REPOT 1
#define SBI_ATT_PAL 2
#define SBI_PLEIN 3
#define SBI_PRISE_D 4
#define SBI_PRISE_G 5
#define SBI_PRISE 6
#define SBI_ROT_1POS 7
#define SBI_ERR_NO_PAL 51

void Init_Gripper();
void State_machine_bg();
void State_machine_bd();
void State_machine_lt();
void State_machine_bi();

#endif
