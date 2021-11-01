/*============================================================\

   Project: Robo_09  -  Gestion Gripper

   Guillaume Galeazzi
   16.02.2009
   Heig-vd/robotique

   gripper_config.h   

 ============================================================

   But:  constantes et mode de la carte gripper

\============================================================*/
#ifndef GRIPPER_CONFIG_HEADER

#define GRIPPER_CONFIG_HEADER 1

#define SIMULATION 1
#define DEBUG 1

// prend en compte la tension de la batterie
#define ADJUST_PWM 0

#define MOT_GAUCHE 1
#define MOT_DROITE 2
#define MOT_LINTEAU 3
#define MOT_BARILLET 4

// gestion des interruptions
#define IRQ_ON (IE |= 0x80)
#define IRQ_OFF (IE &= ~0x80)

// Pour le SPI
#define SPI_FAST 1
// Fspi=1MHz si =1,  100kHz si =0

#define SPI_FREE 0
#define SPI_BSY 1

#define SPI_DSP_BRAS 0
#define SPI_DSP_BILT 1

#define MOT2BRAS MOT_DROITE
// ce que j'appel le moteur 2 sur le DSP des bras est celui du bras gauche
#define MOT2BILT MOT_BARILLET
// ce que j'appel le moteur 2 sur le DSP du barillet et du linteau est celui du bras gauche

#define SPIBSY (SPI0CFG & 0x80)

// Pour l'I2C
#define I2C_GRIPPER_ADD 0x57

// DEFINITION DE QUELQUES CONSTANTE m�canique
#define X_BRAS_HOME 1
#define X_BRAS_PRISE_BARILLET 1
#define X_BRAS_PRISE_BARILLET_UP 1
#define X_BRAS_POSE_LT 1
#define X_BRAS_POSE_LT_UP 1
#define X_BRAS_POSE_LT_UP_UP 1
#define X_BRAS_POSE_SOL 1
#define X_BRAS_POSE_VER 1
#define X_BRAS_TOWER_1PAL 1
#define X_BRAS_TOWER_MAX 1
#define NB_MAX_PAL_TOWER 1
#define V_SLOW 1
#define V_MED 1
#define V_FAST 1
#define C_SLOW 1
#define C_MED 1
#define C_HARD 1
#define BARILLET_HOME 1
#define BARILLET_1POS 1
#define BARILLET_DEMI_POS 1
#define BARILLET_OPEN1 1
// #define BARILLET_OPEN2        1
// #define BARILLET_OPEN3        1
// #define BARILLET_OPEN4        1
#define BARILLET_CLOSE1 1
// #define BARILLET_CLOSE2       1
// #define BARILLET_CLOSE3       1
// #define BARILLET_CLOSE4       1
#define LINTEAU_HOME 1
#define LINTEAU_IN 1
#define LINTEAU_OUT 1

// DEFINITION DE QUELQUES ADDRESSE DSP
/*
Add.    data                    comment
 0      XXXX'XXXX XXXX'XXXX     angle moteur 1
 1      XXXX'XXXX XXXX'XXXX     vitesse moteur 1
 2      XXXX'XXXX XXXX'XXXX     couple moteur 1
 3      XXXX'XXXX XXXX'XXXX     angle moteur 2
 4      XXXX'XXXX XXXX'XXXX     vitesse moteur 2
 5      XXXX'XXXX XXXX'XXXX     couple moteur 2
 6      XXX-'---- XXX-'----     set   config
 7      XXX-'---- XXX-'----     clear config
       {idem pr 2}���� ����
                  ���� ���+-    Led moteur 1 (1=on,     0=off) 
                  ���� ��+--    Led moteur 1 (1=remote, 0=auto)
                  ���� �+---
                  ���� +----  
                  ����
                  ���+------ Clear status error
                  ��+------- Stop mouvement moteur 1
                  �+-------- Start mouvement moteur 1
                  +--------- Homing moteur 1
                  
   Return value:

1    0000'0000 0000'0000
2    000X'XXXX 000X'XXXX     status
     {idem2}   ���� ����
               ���� ���+- Erreur moteur 1    
               ���� ��+-- Couple d�pass� moteur 1
               ���� �+--- Position atteinte moteur 1
               ���� +---- Homing done 
               ����
               ���+------ Free
               ��+------- Led moteur 1 (1=on,     0=off)
               �+-------- n/a
               +--------- n/a
*/
#define SPI_MSK_STATUS 0x1F

#define SPI_CLR_ERR 0x10
#define MOT_STOP 0x20
#define MOT_START 0x40
#define MOT_HOME 0x80

#define ADD_ANG_M1 1
#define ADD_VIT_M1 2
#define ADD_CPL_M1 3
#define ADD_ANG_M2 4
#define ADD_VIT_M2 5
#define ADD_CPL_M2 6
#define ADD_SET_CFG 7
#define ADD_CLR_CFG 8
#define ADD_STATUS 9

#define ADD_OFFSETM1M2 3

#endif

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
