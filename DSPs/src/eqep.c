/**
 * @title   MotionBoard project v2.0
 * @file    eqep.c
 * @brief   Enhanced quadrature encoder peripheral configuration
 * @author  Yves Chevallier <yves.chevallier@kalios.ch>
 * @svn     $Id: eqep.c 585 2009-02-01 12:59:26Z ychevall $
 */
                                                                           
////////////////////////////////////////////////////////////////////////////////
/// Description.                                                                
/// Le module EQep intégré dans la famille des DSP texas 320F280x permet de     
/// traiter les signaux reçus par des codeurs de position avec signaux en       
/// quadrature. Ce fichier contient les routines d'initialisation de ce périph- 
/// érique ainsi que les routines permettant le calcul de la position, des      
/// angles mechanique et électrique ainsi que la vitesse. Un méchanisme         
/// d'interpolation permet d'obtenir une plus grande résolution sur la vitesse. 
////////////////////////////////////////////////////////////////////////////////
#include "global.h"    
#include "eqep.h"
#include "gpio.h"
#include "iq_math/IQmathLib.h"

////////////////////////////////////////////////////////////////////////////////
/// Initialize GPIO for EQep1.                                                  
////////////////////////////////////////////////////////////////////////////////
void EQep1_InitGpio()
{
  EALLOW;
  GpioCtrlRegs.GPAMUX2.bit.GPIO20 = MUX_GPIO20_EQEP1A; 
  GpioCtrlRegs.GPAMUX2.bit.GPIO21 = MUX_GPIO21_EQEP1B;  
  GpioCtrlRegs.GPAMUX2.bit.GPIO23 = MUX_GPIO23_EQEP1I; 
  EDIS;  
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize GPIO for EQep2.                                                  
////////////////////////////////////////////////////////////////////////////////
void EQep2_InitGpio()
{
  EALLOW;
  GpioCtrlRegs.GPAMUX2.bit.GPIO24 = MUX_GPIO24_EQEP2A;  
  GpioCtrlRegs.GPAMUX2.bit.GPIO25 = MUX_GPIO25_EQEP2B; 
  GpioCtrlRegs.GPAMUX2.bit.GPIO26 = MUX_GPIO26_EQEP2I;  
  EDIS;  
}   

////////////////////////////////////////////////////////////////////////////////
/// Initialize GPIO for both EQep modules                                       
////////////////////////////////////////////////////////////////////////////////
void
EQep_InitGpio()
{
	EQep1_InitGpio();
	EQep2_InitGpio();
}

////////////////////////////////////////////////////////////////////////////////
/// Common initialisation for all EQep modules                                  
/// Le module est tout d'abord désactivé avant d'être configuré. Ceci permet d' 
/// éviter des problèmes lors des phases de debug. Le module est ensuite        
/// configuré en mode Quadrature. Les signaux A et B sont interprêté dans une   
/// Machine d'état pour fournir une information de direction et de position     
/// Ce mode permet d'accroitre la précision du codeur par un facteur 4. Le      
/// compteur de position (sur 32 bits) n'est pas remis à zero automatiquement   
/// En revanche lorsque le signal d'index est détecté, la position courante est 
/// copiée dans le registre QPOSILAT. Lorsque ce registre est lu par le soft,   
/// les valeurs de QCTMR et QCPRD sont également latchées. Elles permettent l'  
/// interpolation de la vitesse permettant à celle-ci d'être moins granuleuse   
/// Ceci dans le but d'accroitre au maximuk le gain de la boucle de vitesse qui 
/// doit être le plus élevé possible pour maximiser les performances de la      
/// boucle de position.                                                         
////////////////////////////////////////////////////////////////////////////////
void 
EQep_InitCommonRegisters(volatile struct EQEP_REGS *EQepRegsP)
{
  volatile struct EQEP_REGS *EQepRegs; 

  EQepRegs = EQepRegsP; 

  EQepRegs->QEPCTL.bit.QPEN=0; 	  	// QEP enable
  EQepRegs->QCAPCTL.bit.CEN=0;      // QEP Capture Enable

  EQepRegs->QDECCTL.bit.QSRC=0;		// Quadrature mode enabled
  EQepRegs->QEPCTL.bit.FREE_SOFT=2; // Position counter is unaffected by emulation suspend
  EQepRegs->QEPCTL.bit.PCRM=1;	    // Position counter reset on maximum position
  EQepRegs->QEPCTL.bit.QCLM=0; 	    // Latch QCTMRLAT and QCPRDLAT on QPOSCNT reading
  EQepRegs->QEPCTL.bit.IEL=1;       // Latch Position on index event

  EQepRegs->QPOSMAX=0xFFFFFFFF;     // QPOSMAX receive maximum value as possible
  EQepRegs->QEPCTL.bit.QPEN=1; 	  	// QEP enable

  EQepRegs->QCAPCTL.bit.UPPS=0;   	// UPEVENT = QCLK/1
  EQepRegs->QCAPCTL.bit.CCPS=0;	    // CAPCLK = SYSCLK/1
  EQepRegs->QCAPCTL.bit.CEN=1; 	    // QEP Capture Enable

  EQepRegs->QPOSCNT = 0; 			// Reset position counter
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize both EQep                                                        
////////////////////////////////////////////////////////////////////////////////
void 
EQep_InitRegisters()
{
	EQep_InitCommonRegisters(&EQep1Regs);
	EQep_InitCommonRegisters(&EQep2Regs);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize EQep                                                            
////////////////////////////////////////////////////////////////////////////////
void 
Init_EQep ()
{
	EQep_InitRegisters();
 	EQep_InitGpio();
}

////////////////////////////////////////////////////////////////////////////////
/// EQep Reset position
////////////////////////////////////////////////////////////////////////////////   
void EQep_reset_position(eqep_handle p)
{
	p->position = 0; 
}

////////////////////////////////////////////////////////////////////////////////
/// EQep Position Measurement                                                   
////////////////////////////////////////////////////////////////////////////////
#define oldpos p->oldposition
#define middlecount (EQepRegs->QPOSMAX/2)
void
EQep_process(eqep_handle p)
{
  volatile struct EQEP_REGS *EQepRegs; // Thanks to MSL for this good trick
  volatile unsigned long curpos; 

  // Save EQep pointer in a local pointer
  EQepRegs = p->EQepRegs; 

  // What is direction ?
  p->direction  = EQepRegs->QEPSTS.bit.QDF;

  // Count revolution and Check an index occurence
  if (EQepRegs->QFLG.bit.IEL == 1) 
  {
    EQepRegs->QCLR.bit.IEL = 1;
    p->index_sync_flag = 1;
    p->revolution += (p->direction)?1:-1;  
  }

  // Compute delta position
  curpos = EQepRegs->QPOSCNT; 
  if(oldpos > curpos && oldpos > middlecount && curpos < middlecount)
    p->delta_pos = EQepRegs->QPOSMAX - oldpos + curpos;
  else if(oldpos > curpos)
    p->delta_pos = -(oldpos - curpos);
  else if(curpos > middlecount && oldpos < middlecount)
    p->delta_pos = -(EQepRegs->QPOSMAX - curpos + oldpos);
  else 
    p->delta_pos = curpos - oldpos; 

  p->oldposition = curpos; 

  // Compute position 
  p->position += p->delta_pos; 
}

////////////////////////////////////////////////////////////////////////////////
/// EQep Velocity Measurement                                                   
////////////////////////////////////////////////////////////////////////////////
void
EQep_velocity (eqep_handle p)
{
  #define SPEED_SCALE 131
  volatile struct EQEP_REGS *EQepRegs;
  volatile long curpos;  
  long velo = 0; 
  long veloFine = 0;   

  // Save EQep pointer in a local pointer
  EQepRegs = p->EQepRegs; 
    
  // Compute position directly from coder ticks
  curpos = p->position;
  velo = (curpos - p->oldposition2) * SPEED_SCALE;
  p->oldposition2 = curpos;
  
  // Compte fine position from EQep p
  /*
  if(EQepRegs->QEPSTS.bit.COEF == 0)
    veloFine = _IQmpy(_IQdiv(EQepRegs->QCTMRLAT, EQepRegs->QCPRDLAT), _IQ(0.00366)); 
  else
    veloFine = SPEED_SCALE;
*/
  // If direction has changed fine position can't be computed
  if(EQepRegs->QEPSTS.bit.CDEF == 1)
    veloFine = 0; 

  // The final position is base position plus fine position
  p->velocity = velo + veloFine; 

  // Finally the event clag is cleared
  EQepRegs->QEPSTS.all=0x88;			
}

////////////////////////////////////////////////////////////////////////////////
/// Speed & Position Calculation.                                               
////////////////////////////////////////////////////////////////////////////////

