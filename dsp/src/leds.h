
static inline void Init_leds(void);

static inline void Led1(char value);
static inline void Led2(char value);

static inline void Led1_toggle(void);
static inline void Led2_toggle(void);
// Initialize leds
static inline void
Init_leds(void)
{
  EALLOW;

  // No peripheral assigned
  GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;
  GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;

  // Set direction to output
  GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;
  GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;

  // Clear leds
  GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
  GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;

  EDIS;
}
// Control red led.
// @param value State of the red led
static inline void
Led1(char value)
{
  GpioDataRegs.GPADAT.bit.GPIO28 = value;
}
// Control red led.
// @param value State of the red led
static inline void
Led2(char value)
{
  GpioDataRegs.GPADAT.bit.GPIO30 = value;
}
// Toggle red led.
static inline void
Led1_toggle()
{
  GpioDataRegs.GPATOGGLE.bit.GPIO28 = 1;
}
// Toggle red led.
static inline void
Led2_toggle()
{
  GpioDataRegs.GPATOGGLE.bit.GPIO28 = 1;
}
