#ifndef _I2C_H_
#define _I2C_H_

#include "types.h"


#define ENABLE_IRQ (IE |= 0x80)
#define NULL 0

/*#define SDCC_I2C_IRQ  void i2c_irq(void) interrupt 7          \
                      {                                       \
                         i2c_isr();                           \
                      }                                       \
                                                              \
                      void i2c_timeout_irq(void) interrupt 14 \
                      {                                       \
                        timer_scl_low_timeout_isr();          \
                      }    */                                   


void i2c_init(uint8 address,void * ptr, uint8 size);
void i2c_set_callback_mode(void (*function)(uint8 reg, uint8 *buffer)); 
int8 i2c_is_busy(void);

void timer_scl_low_timeout_isr(void);
void i2c_isr(void);


#endif
