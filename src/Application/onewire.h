#ifndef ONEWIRE_H_
#define ONEWIRE_H_
#include <stdint.h>

#define OWPORTDIR P2DIR
#define OWPORTOUT P2OUT
#define OWPORTIN P2IN
#define OWPORTREN P2REN
#define OWPORTPIN BIT3
#define OW_LO {	OWPORTDIR |= OWPORTPIN;	OWPORTREN &= ~OWPORTPIN; OWPORTOUT &= ~OWPORTPIN; }
#define OW_HI {	OWPORTDIR |= OWPORTPIN;	OWPORTREN &= ~OWPORTPIN; OWPORTOUT |= OWPORTPIN; }
#define OW_RLS { OWPORTDIR &= ~OWPORTPIN; OWPORTREN |= OWPORTPIN; OWPORTOUT |= OWPORTPIN; }


#define DS1820_SKIP_ROM             0xCC
#define DS1820_READ_SCRATCHPAD      0xBE
#define DS1820_CONVERT_T            0x44

//########################################################################

int onewire_reset();
void onewire_write_bit(int bit);
int onewire_read_bit();
void onewire_write_byte(uint8_t byte);
uint8_t onewire_read_byte();
void onewire_line_low();
void onewire_line_high();
void onewire_line_release();
unsigned int GetData(void);
#endif /* ONEWIRE_H_ */
