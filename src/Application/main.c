/**
 *  @file main.c
 *
 *  @brief Application main
 *
 *  @version  1.0
 *
 *  @attention IMPORTANT: Your use of this Software is limited to those specific 
 *             rights granted under the terms of a software license agreement 
 *             between the user who downloaded the software, his/her employer 
 *             (which must be your employer) and Anaren (the "License"). You may
 *             not use this Software unless you agree to abide by the terms of 
 *             the License. The License limits your use, and you acknowledge,
 *             that the Software may not be modified, copied or distributed unless
 *             in connection with an authentic Anaren product. Other than for the 
 *             foregoing purpose, you may not use, reproduce, copy, prepare 
 *             derivative works of, modify, distribute, reverse engineer, decompile,
 *             perform, display or sell this Software and/or its documentation 
 *             for any purpose. 
 *             YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION
 *             ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS 
 *             OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY  WARRANTY OF 
 *             MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR 
 *             PURPOSE. IN NO EVENT SHALL ANAREN OR ITS LICENSORS BE LIABLE OR
 *             OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 *             BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR 
 *             INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY 
 *             INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, 
 *             LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS,
 *             TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT
 *             NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS. 
 */

#include <msp430g2553.h>
#include <stdint.h>
#include "onewire.h"
#include "delay.h"
#include "Application.h"
#include "Callback.h"
//unsigned int getADC();
void ow_portsetup();
unsigned int userBuffer[4] = { 0 };            //!< Data buffer defined by user.
//userBuffer
/**
 * @fn void main()
 *
 * @brief main 
 */

/*--------------------------------------------------------*/
void main() {
	InitSystem();           // Initialize peripherals, application and protocol.
	ow_portsetup();
	while(1)
	  {
	    // ---------- Get data from hardware-----------
	    if (pushButtonAction == ACTIVATE_ALARM) {                                   // Check if Pushbutton pressed and toggle RedLED accordingly.
	      pushButtonAction = 0;
	      LEDToggle(__BSP_LEDRED1);
	    }
	    userBuffer[2] = LEDState(__BSP_LEDRED1);                                    // Get RedLED state(from hardware) and store it to user buffer.


	    // ----------- Get the data from the GUI
	    #ifdef __COMMAND_INTERFACE
	    unsigned char id = 0xFF;                                                    // This is only to get the data from the GUI corresponding to the nodeID.
	      if(CmdIFProcess(&id, &userBuffer[0])){                                    // Run the command interface to get the data(if any) from GUI.
	        if(id != 0xFF){
	          MapControlsToHW(id, &userBuffer[0]);                                  // Map the received data to the node's respective structure and to its own hardware(if needed).
	          if(userBuffer[1] & 0x0001){                                           // Check if a command is received to set my own green LED.
	            LEDOn(__BSP_LEDGREEN1);
	          }
	          else{
	            LEDOff(__BSP_LEDGREEN1);
	          }
	        }
	      }
	    #endif


	    // ---------- Get data from hardware-----------
	    if (ADCConversionDone){                                                    // Check for new analog data (local temperature).
	      ADCConversionDone = 0;                                                   // Clear flag.
	      userBuffer[0] = GetData();//                        // Get temperature data from ADC(hardware) and store it to user buffer.
	      #ifdef __COMMAND_INTERFACE
	        MapSensorsFromHW(0, &userBuffer[0]);                                   // Get local only hardware data to send it to the paired module.
	        ReportData(0);                                                         // Report Local only hardware states to GUI(temperature and RedLED)
	      #endif

	      // ---------- Send data to radio (SENSOR)-----------
	      if(ApplicationState == SENSOR){                                          // We only want to send data from a sensor node when an ADC conversion is complete.
	        WirelessOperation(&userBuffer[0]);
	        if(userBuffer[1] & 0x0001){                                            // Check if a command is received to set my own green LED.
	          LEDOn(__BSP_LEDGREEN1);
	        }
	        else{
	          LEDOff(__BSP_LEDGREEN1);
	        }
	      }
	    }

	    // ---------- Get data from radio (HUB)-----------
	    if(ApplicationState == HUB){                                               // Check for data received from a sensor node every pass through the loop.
	      WirelessOperation(&userBuffer[0]);
	      if(userBuffer[1] & 0x0001){                                              // Check if a command is received to set my own green LED.
	        LEDOn(__BSP_LEDGREEN1);
	      }
	      else{
	        LEDOff(__BSP_LEDGREEN1);
	      }
	    }


	    // ------------ Go to sleep if there is nothing else to do -------------
	    __BSP_LOW_POWER_MODE(LOW_POWER_MODE);                                      // Nothing else to do.  Sleep until next interrupt arrives.
	  }
}

/*unsigned int getADC() {

 unsigned int result = 0;

 result = (ADCGetConversionValue(__BSP_ADC1));
 if (result & 1000000000000000) {

 result = result ^ 1000000000000000;

 //result=~result;
 }
 return result;
 }*/

void ow_portsetup() {
	OWPORTDIR |= OWPORTPIN;
	OWPORTOUT |= OWPORTPIN;
	OWPORTREN |= OWPORTPIN;
}

unsigned int ReadDS1820(void) {
	unsigned int i;
	uint16_t byte = 0;
	for (i = 16; i > 0; i--) {
		byte >>= 1;
		if (onewire_read_bit()) {
			byte |= 0x8000;
		}
	}
	return byte;
}

unsigned int GetData(void) {

	uint16_t temp;
	onewire_reset();
	onewire_write_byte(0xcc); // skip ROM command
	onewire_write_byte(0x44); // convert T command
	OW_HI
	DELAY_MS(750);
	// at least 750 ms for the default 12-bit resolution
	onewire_reset();
	onewire_write_byte(0xcc); // skip ROM command
	onewire_write_byte(0xbe); // read scratchpad command
	temp= ReadDS1820();
	temp=temp<<4;
	return temp;
	/*if (temp < 0x8000) {
	 return temp; //(temp * 0.0625);
	 } else {
	 temp = (~temp) + 1;}
	//return temp; //(temp * 0.0625);*/
}

int onewire_reset() {
	OW_LO
	DELAY_US(480);
	// 480us minimum
	OW_RLS
	DELAY_US(40);
	// slave waits 15-60us
	if (OWPORTIN & OWPORTPIN)
		return 1; // line should be pulled down by slave
	DELAY_US(300);
	// slave TX presence pulse 60-240us
	if (!(OWPORTIN & OWPORTPIN))
		return 2; // line should be "released" by slave
	return 0;
}

//#####################################################################

void onewire_write_bit(int bit) {
//  DELAY_US(1); // recovery, min 1us
	OW_HI
	if (bit) {
		OW_LO
		DELAY_US(5);
		// max 15us
		OW_RLS
			// input
		DELAY_US(56);
	} else {
		OW_LO
		DELAY_US(60);
		// min 60us
		OW_RLS
			// input
		DELAY_US(1);
	}
}

//#####################################################################

int onewire_read_bit() {
	int bit = 0;
//  DELAY_US(1); // recovery, min 1us
	OW_LO
	DELAY_US(5);
	// hold min 1us
	OW_RLS
	DELAY_US(10);
	// 15us window
	if (OWPORTIN & OWPORTPIN) {
		bit = 1;
	}
	DELAY_US(46);
	// rest of the read slot
	return bit;
}

//#####################################################################

void onewire_write_byte(uint8_t byte) {
	int i;
	for (i = 0; i < 8; i++) {
		onewire_write_bit(byte & 1);
		byte >>= 1;
	}
}

//#####################################################################

uint8_t onewire_read_byte() {
	unsigned int i;
	uint8_t byte = 0;
	for (i = 0; i < 8; i++) {
		byte >>= 1;
		if (onewire_read_bit())
			byte |= 0x80;
	}
	return byte;
}
