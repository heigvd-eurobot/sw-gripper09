/**
 * @title   MotionBoard project v2.0
 * @file    pwm.c
 * @brief   Pulse with modulation module
 * @author  Yves Chevallier <yves.chevallier@kalios.ch>
 * @svn     $Id: pwm.c 531 2008-12-14 20:35:55Z ychevall@heig-vd.ch $
 */

// Includes files

#include "global.h" // Device Headerfile and Examples Include File
#include "pwm.h"

// Global constants (Configuration PWM modules)

#define PWM_FREQUENCY 24e3 // 24 kHz
#define DEAD_TIME 100e-9   // 100ns
#define ADC_WINDOW 700e-9  // 700ns

#define SP (float)(1.0 / (CPU_RATE * 1.0e-9 * PWM_FREQUENCY * 2.0))
#define DT (float)(DEAD_TIME * 1.0e9 / 10.0)
#define AW (float)(ADC_WINDOW * 1.0e9 / 2.0 / 10.0)
#define HAW (float)(AW / 2.0)
#define HSP (float)(SP / 2.0)

// Global Variables
float sp = SP;
float dt = DT;
float aw = AW;
float haw = HAW;
float hsp = HSP;

// Initialize Gpio for ePWM1
void EPwm1_ResetGpio(void)
{
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO0 = EN_PULLUP; // Enable pull-up on GPIO0 (EPWM1A)
  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;        // Configure GPIO0 as EPWM1A
  EDIS;
}

// Initialize Gpio for ePWM2
void EPwm2_ResetGpio(void)
{
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO2 = EN_PULLUP; // Enable pull-up on GPIO2 (EPWM2A)
  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;        // Configure GPIO2 as EPWM2A
  EDIS;
}

// Initialize Gpio for ePWM
void EPwm_ResetGpio()
{
  EPwm1_ResetGpio(); // Configure PWM 1 for motor A
  EPwm2_ResetGpio(); // Configure PWM 2 for motor A
}

// Initialize Gpio for ePWM1
void EPwm1_InitGpio(void)
{
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO0 = EN_PULLUP; // Enable pull-up on GPIO0 (EPWM1A)
  //GpioCtrlRegs.GPAPUD.bit.GPIO1 = EN_PULLUP; // Enable pull-up on GPIO1 (EPWM1B)
  GpioCtrlRegs.GPAMUX1.bit.GPIO0 = MUX_GPIO0_EPWM1A; // Configure GPIO0 as EPWM1A
  //GpioCtrlRegs.GPAMUX1.bit.GPIO1 = MUX_GPIO1_EPWM1B; // Configure GPIO1 as EPWM1B
  EDIS;
}

// Initialize Gpio for ePWM2
void EPwm2_InitGpio(void)
{
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO2 = EN_PULLUP; // Enable pull-up on GPIO2 (EPWM2A)
  //GpioCtrlRegs.GPAPUD.bit.GPIO3 = EN_PULLUP; // Enable pull-up on GPIO3 (EPW2B)
  GpioCtrlRegs.GPAMUX1.bit.GPIO2 = MUX_GPIO2_EPWM2A; // Configure GPIO2 as EPWM2A
  //GpioCtrlRegs.GPAMUX1.bit.GPIO3 = MUX_GPIO3_EPWM2B; // Configure GPIO3 as EPWM2B
  EDIS;
}

// Initialize Gpio for ePWM Trip
void EPwm_InitTripGpio(void)
{
  /*
  EALLOW;
  GpioCtrlRegs.GPAPUD.bit.GPIO12 = EN_PULLUP;        // Enable pull-up on GPIO12 (TZ1)
  GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = GPXQSELX_ASYNC; // Asynch input GPIO12 (TZ1)
  GpioCtrlRegs.GPAMUX1.bit.GPIO12 = MUX_GPIO12_TZ1;  // Configure GPIO12 as TZ1
  EDIS;   */
}

// Initialize Gpio for ePWM
void EPwm_InitGpio()
{
  EPwm1_InitGpio(); // Configure PWM 1 for motor A
  EPwm2_InitGpio(); // Configure PWM 2 for motor A

  EPwm_InitTripGpio(); // Configure TRIP inputs
}

// Configure PWM1 register
// For this PWM, the initialization is fully commented just for enjoy the
// curious programmer...
void EPwm1_ConfigureRegisters()
{
  // Counter-Compare Submodule Registers //
  {
    // Counter-Compare A Register
    EPwm1Regs.CMPA.half.CMPA = SP;

    // Counter-Compare B Register
    EPwm1Regs.CMPB = ADC_SOC; // Used for ADC start-of-conversion
  }
  // Action-Qualifier Submodule Registers //
  {
    // Action-Qualifier Output A Control Register
    EPwm1Regs.AQCTLA.bit.CAU = 1; // Output high when the counter is incrementing
    EPwm1Regs.AQCTLA.bit.CAD = 2; // Output low when the counter is decrementing
  }
  // Death-Band Submodule Registers      //
  {
    // Death-Band Generator Control Register
    EPwm1Regs.DBCTL.bit.POLSEL = 1;
    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;
    EPwm1Regs.DBCTL.bit.IN_MODE = 0;

    // Death-Band Generator Rising Edge Delay Register
    EPwm1Regs.DBRED = DT; // Risingg Edge Delay Count (10 bit counter)

    // Death-Band Generator Falling Edge Delay Register
    EPwm1Regs.DBFED = DT; // Falling Edge Delay Count (10 bit counter)
  }
  // PWM-Chopper Submodule Control Register //
  {
    // PWM-Chopper Control Register
    EPwm1Regs.PCCTL.bit.CHPEN = 0; // PWM-Chopper is disabled
  }
  // Trip-Zone Submodule Control and Status Registers //
  {
    EALLOW;

    // Trip-Zone Select Register
    //EPwm1Regs.TZSEL.bit.OSHT1 = 1;     // Enable TZ1 as a one-shot trip source
    //EPwm1Regs.TZSEL.bit.OSHT2 = 1;      // Enable TZ2 (12V gates) as a one-shot trip source

    // Trip-Zone Control Register
    EPwm1Regs.TZCTL.bit.TZA = 1; // Force EPWMxA to a low state if trip event occurs
    EPwm1Regs.TZCTL.bit.TZB = 1; // Force EPWMxB to a low state if trip event occurs

    // Trip-Zone Enable Interupt Register
    EPwm1Regs.TZEINT.bit.OST = 0; // Disable one-shot interrupt generation
    EPwm1Regs.TZEINT.bit.CBC = 0; // Disable cycle-by-cycle interrupt generation

    // Trip-Zone Clear Register
    EPwm1Regs.TZCLR.bit.CBC = 1; // Clear this Trip condition
    EPwm1Regs.TZCLR.bit.OST = 1; // Clear this Trip condition
    EPwm1Regs.TZCLR.bit.INT = 1; // Clear this Trip-interrupt flag

    EDIS;
  }
  // Time-Base Configuration             //
  {
    // Time-Base Control Register
    EPwm1Regs.TBCTL.bit.PHSEN = 0;     // Master module
    EPwm1Regs.TBCTL.bit.PHSDIR = 0;    // Count down after the synchronization event
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 1;  // Synchro. Out. when CTR = zero
    EPwm1Regs.TBCTL.bit.PRDLD = 1;     // Directly accesses to the TBPRD active Register
    EPwm1Regs.TBCTL.bit.CTRMODE = 2;   // Up-down count mode
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 3; // No stop when emulation is enabled

    // Time-Base Status Register
    EPwm1Regs.TBSTS.bit.CTRMAX = 1; // Clear the latched event
    EPwm1Regs.TBSTS.bit.SYNCI = 1;  // Clear the latched event

    // Time-Base Period Register
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // TBCLK = SYSCLKOUT = 100MHz
    EPwm1Regs.TBPRD = SP;              // Based on CLK

    // Time-Base Phase Register
    EPwm1Regs.TBPHS.half.TBPHS = 0;

    // Time-Base Counter Register
    EPwm1Regs.TBCTR = 0;
  }
  // Event-Trigger Submodule Registers   //
  {
    // Event-Trigger Prescale Register
    EPwm1Regs.ETPS.bit.SOCAPRD = 1; // Generate SOCA event on the first event ETPS
    EPwm1Regs.ETPS.bit.INTPRD = 1;  // Generate interrupt on the first event

    // Event-Trigger Clear Register
    EPwm1Regs.ETCLR.bit.SOCA = 1; // Clear ETFLG[SOCA] flag bit
    EPwm1Regs.ETCLR.bit.SOCB = 1; // Clear ETFLG[SOCB] flag bit
    EPwm1Regs.ETCLR.bit.INT = 1;  // Clear ETFLG[INT] flag bit

    // Event-Trigger Force Register
    EPwm1Regs.ETFRC.all = 0; // Rajout

    // Event-Trigger Flag Register
    EPwm1Regs.ETFLG.all = 0; // Rajout

    // Event-Trigger Selection Register
    EPwm1Regs.ETSEL.bit.SOCASEL = 7; // Enable event: time-base counter equal to CMPB when the timer is decrementing
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  // Enable EPWMxSOCA pulse

    EPwm1Regs.ETSEL.bit.INTSEL = 1; // Enable event time-base counter equal to zero (TBCTR=0x0000)
    EPwm1Regs.ETSEL.bit.INTEN = 1;  // Enable EPWMx_INT generation
  }
}

// Common initialization for slave PWM channels
void EPwmSlave_ConfigureRegisters(volatile struct EPWM_REGS *EPwmSlaveRegs)
{

  EPwmSlaveRegs->CMPA.half.CMPA = SP; // 50% Duty Cycle
  EPwmSlaveRegs->CMPB = 0;            // Not-used
  EPwmSlaveRegs->AQCTLA.bit.CAU = 1;  // Output high when the counter is incrementing
  EPwmSlaveRegs->AQCTLA.bit.CAD = 2;  // Output low when the counter is decrementing
  EPwmSlaveRegs->DBCTL.bit.POLSEL = 1;
  EPwmSlaveRegs->DBCTL.bit.OUT_MODE = 3;
  EPwmSlaveRegs->DBCTL.bit.IN_MODE = 0;
  EPwmSlaveRegs->DBRED = DT;          // Risingg Edge Delay Count (10 bit counter)
  EPwmSlaveRegs->DBFED = DT;          // Falling Edge Delay Count (10 bit counter)
  EPwmSlaveRegs->PCCTL.bit.CHPEN = 0; // PWM-Chopper is disabled

  EALLOW;
  //EPwmSlaveRegs->TZSEL.bit.OSHT1 = 1;      // Enable TZ1 as a one-shot trip source
  //EPwmSlaveRegs->TZSEL.bit.OSHT2 = 1;      // Enable TZ2 (12V gates) as a one-shot trip source
  EPwmSlaveRegs->TZCTL.bit.TZA = 1; // Force EPWMxA to a low state if trip event occurs
  EPwmSlaveRegs->TZCTL.bit.TZB = 1; // Force EPWMxB to a low state if trip event occurs
  EDIS;

  EPwmSlaveRegs->TZEINT.bit.OST = 0;      // Disable one-shot interrupt generation
  EPwmSlaveRegs->TZEINT.bit.CBC = 0;      // Disable cycle-by-cycle interrupt generation
  EPwmSlaveRegs->TZCLR.bit.CBC = 1;       // Clear this Trip condition
  EPwmSlaveRegs->TZCLR.bit.OST = 1;       // Clear this Trip condition
  EPwmSlaveRegs->TZCLR.bit.INT = 1;       // Clear this Trip-interrupt flag
  EPwmSlaveRegs->TBCTL.bit.PHSEN = 1;     // Slave module
  EPwmSlaveRegs->TBCTL.bit.PHSDIR = 1;    // Count down after the synchronization event
  EPwmSlaveRegs->TBCTL.bit.SYNCOSEL = 0;  // Sync flow-through
  EPwmSlaveRegs->TBCTL.bit.PRDLD = 1;     // Directly accesses to the TBPRD active Register
  EPwmSlaveRegs->TBCTL.bit.CTRMODE = 2;   // Up-down count mode
  EPwmSlaveRegs->TBSTS.bit.CTRMAX = 1;    // Clear the latched event
  EPwmSlaveRegs->TBSTS.bit.SYNCI = 1;     // Clear the latched event
  EPwmSlaveRegs->TBCTL.bit.HSPCLKDIV = 0; // TBCLK = SYSCLKOUT = 100MHz
  EPwmSlaveRegs->TBCTL.bit.FREE_SOFT = 3; // No stop when emulation is enabled
  EPwmSlaveRegs->TBPRD = SP;
  EPwmSlaveRegs->TBPHS.half.TBPHS = 0;
  EPwmSlaveRegs->TBCTR = 0;
  EPwmSlaveRegs->ETFRC.all = 0; // Rajout
  EPwmSlaveRegs->ETFLG.all = 0; // Rajout
}

// Initialize Registers for ePWM
void EPwm_InitRegisters()
{
  EPwm1_ConfigureRegisters();               // Master Module
  EPwmSlave_ConfigureRegisters(&EPwm2Regs); // Slave Module
}

// Initialize PWM Controller
void Init_epwm(void)
{
  EPwm_ResetGpio();
  EPwm_InitRegisters(); // Very Important Remark: This Initialisation must be before GPIO init.
  EPwm_InitGpio();

  EALLOW;
  SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
  EDIS;
}

// Set value for PWM1
// @param dc Duty cycle from 0% to +100% (-1.0..+1.0)  50% = 0
void set_pwm1(_iq16 dc)
{
  if (dc > 0)
  {
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
  }
  else
  {
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    dc = -dc;
  }

  EPwm1Regs.CMPA.half.CMPA = (1 << 11) - (dc >> 5);
}

// Set value for PWM2
// @param dc Duty cycle from 0% to +100% (-1.0..+1.0)  50% = 0
void set_pwm2(_iq16 dc)
{
  if (dc > 0)
  {
    GpioDataRegs.GPASET.bit.GPIO4 = 1;
  }
  else
  {
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
    dc = -dc;
  }

  EPwm2Regs.CMPA.half.CMPA = (1 << 11) - (dc >> 5);
}
