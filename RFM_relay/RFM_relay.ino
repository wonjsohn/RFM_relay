
// Basic RadioHead RFM9x Library test
// Receiver side

// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>

/* for Feather32u4 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
*/

//for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


/* for shield
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 7
*/

/* Feather 32u4 w/wing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     2    // "SDA" (only SDA/SCL/RX/TX have IRQ!)
*/

/* Feather m0 w/wing
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"
*/

#if defined(ESP8266)
/* for ESP w/featherwing */
#define RFM95_CS  2    // "E"
#define RFM95_RST 16   // "D"
#define RFM95_INT 15   // "B"

#elif defined(ESP32)  
/* ESP32 feather w/wing */
#define RFM95_RST     27   // "A"
#define RFM95_CS      33   // "B"
#define RFM95_INT     12   //  next to A

#elif defined(NRF52)  
/* nRF52832 feather w/wing */
#define RFM95_RST     7   // "A"
#define RFM95_CS      11   // "B"
#define RFM95_INT     31   // "C"

#elif defined(TEENSYDUINO)
/* Teensy 3.x w/wing */
#define RFM95_RST     9   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     4    // "C"
#endif


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 403.5//403.5

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define PIN_LED 13  // PA17 
#define PIN_PULSEOUT_TO_CWU  10// PA18 (11, PA16 is damaged in arduino) 

void setup()
{
	// pulseout from the CWU's 2nd RH receiver. This pin will be wired into the PULSE pin of the CWU. 
	pinMode(PIN_PULSEOUT_TO_CWU, OUTPUT);  /// was OUTPUT 20210311. 
	//digitalWrite(PIN_PULSEOUT_TO_CWU, LOW); // default low

	//*  TCC for precision pulse control: 20210312 *//
	//TCC common clock setting 
	GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 10: 48MHz/1=48MHz
		GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
	while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

	GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOWs
		GCLK_GENCTRL_GENEN |         // Enable GCLK4
		GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
		GCLK_GENCTRL_ID(4);          // Select GCLK4
	while (GCLK->STATUS.bit.SYNCBUSY);               // Wait for synchronization

	//*** Using direct port and pin numbers *******

	PORT->Group[g_APinDescription[PIN_PULSEOUT_TO_CWU].ulPort].PINCFG[g_APinDescription[PIN_PULSEOUT_TO_CWU].ulPin].bit.PMUXEN = 1;

	// Connect the TCC timers to the port outputs - port pins are paired odd PMUO and even PMUXE
	// F & E specify the timers: TCC0, TCC1 and TCC2
	PORT->Group[g_APinDescription[PIN_PULSEOUT_TO_CWU].ulPort].PMUX[g_APinDescription[PIN_PULSEOUT_TO_CWU].ulPin >> 1].bit.PMUXE = PORT_PMUX_PMUXE_F_Val; //PA18, TCC0-WO2 (CC[2]),  (PA16, TCC0-WO6 (CC[2]))




	// Feed GCLK4 to TCC0 and TCC1
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
		GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
		GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
	while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization  


	 // TCC0 setting 
	TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;          // Single slope PWM operation     
	while (TCC0->SYNCBUSY.bit.WAVE);                 // Wait for synchronization

	TCC0->PER.reg = 30000;// because of the one-shot mode, PER is not important. 
	while (TCC0->SYNCBUSY.bit.PER);                  // Wait for synchronization
	TCC0->CC[2].reg = 1500;   // pulse width (Isrc_preload_pulseout_width), 1500: 250us.  1200: 200us. 
	while (TCC0->SYNCBUSY.bit.CC2);


	TCC0->CTRLBSET.reg |= TCC_CTRLBSET_ONESHOT;// | TCC_CTRLBSET_DIR;  // Is this right way to do down-counting and one shot mode together? I concur. 
	//last_cmd = TCC_CTRLBSET_DIR;
	while (TCC0->SYNCBUSY.bit.CTRLB);                 // Wait for synchronization

	// Set oneshot operation to output 0V when the timer is stopped on,  
	TCC0->DRVCTRL.reg |= TCC_DRVCTRL_NRE2;    // WO-2?

	TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLhaveK4 by 1 (was 8)
		TCC_CTRLA_PRESCSYNC_RESYNC |   // set to retrigger timer on GCLK with prescaler reset
		TCC_CTRLA_ENABLE;             // Enable the TCC0 output
	while (TCC0->SYNCBUSY.bit.ENABLE);               // Wait for synchronization

	TCC0->CTRLA.bit.ENABLE = 1;                     // Disable the TCC1 counter
	while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization

	SerialUSB.println("TCC0 for oneshot setting completed.");





	pinMode(PIN_LED, OUTPUT);
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);

	SerialUSB.begin(38400);
	//  while (!SerialUSB) {
	//    delay(1);
	//  }
	delay(100);

	SerialUSB.println("Feather LoRa RX Test!");

	// manual reset
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);

	while (!rf95.init()) {
		SerialUSB.println("LoRa radio init failed");
		SerialUSB.println("Uncomment '#define SerialUSB_DEBUG' in RH_RF95.cpp for detailed debug info");
		while (1);
	}
	SerialUSB.println("LoRa radio init OK!");

	// Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
	if (!rf95.setFrequency(RF95_FREQ)) {
		SerialUSB.println("setFrequency failed");
		while (1);
	}
	SerialUSB.print("Set Freq to: "); SerialUSB.println(RF95_FREQ);

	//rf95.setPromiscuous(true);/// ? // need to setPromiscuous(true) to receive all frames (not sure)


	// default setModeRx in recv seems to be configured to RXCONTINUOUSMODE 

	 // receive single (RXSINGLE)

	rf95.setPreambleLength(6); // Default is 8 , setting to 6 doesn't not seem to increase rate

	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	rf95.setTxPower(5, false);


	// setting method 1 (Sets all the registers required to configur, e.g. BW, SF) 
  // option: https://github.com/Yveaux/RadioHead/blob/master/RadioHead/RH_RF95.cpp#L502
//  const RH_RF95::ModemConfig myProfile =  { 
//    RH_RF95_BW_500KHZ | RH_RF95_CODING_RATE_4_5,
//    RH_RF95_SPREADING_FACTOR_128CPS }; //apparently can't handle RH_RF95_SPREADING_FACTOR_64CPS 
////RH_RF95_IMPLICIT_HEADER_MODE_ON
//  rf95.setModemRegisters(&myProfile);

//// Set up FIFO 
//    // We configure so that we can use the entire 256 byte FIFO for either receive
//    // or transmit, but not both at the same time
//  rf95.spiWrite(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
//  rf95.spiWrite(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

	//rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128); /// works  ~ 100Hz. try with setting SF. CRC.

	//https://hackaday.io/project/27791/instructions
	RH_RF95::ModemConfig myconfig = { 0x92,   0x70,    0x00 };
	// CR:2->e, doesn't make it faster. 2->1 doesn't received (implicit header mode): 0x92 is final.
	// SF:7->6 doesn't receive. (pulse rate halves)  -: 70 is final. 
	// 00: no CRC, final.

	rf95.setModemRegisters(&myconfig);

	//rf95.spiWrite(0x31, 0b101); /// followinrecommenation preamble length = 6  (why doesn't this work)
	//rf95.spiWrite(0x37, 0x0C);/// following recommenation preamble length = 6  (why doesn't this work)
	////It's been a while, but if I remember correctly the device I tested with would only use implicit header mode with low data rate enabled. 




	//rf95.spiWrite(    );
	//rf95.printRegisters();
}


int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
	//SerialUSB.println("loop 1st");
	//digitalWrite(PIN_PULSEOUT_TO_CWU, LOW);

	if (rf95.available())
	{
		//SerialUSB.println("rf95 available");
		// Should be a message for us now
		uint8_t buf[1];
		uint8_t len = sizeof(buf);

		if (rf95.recv(buf, &len))
		{
			digitalWrite(PIN_LED, HIGH);
			//RH_RF95::printBuffer("Received: ", buf, len);// this one has a problem.
			packetnum++;

			SerialUSB.print("Got: ");
			//      SerialUSB.print(packetnum);
				 // SerialUSB.print(" ");
			SerialUSB.println((char*)buf);
			SerialUSB.print("RSSI: ");
			//RSSI: receiver signal strength indicator. This number will range from about -15 to about -100. The larger the number (-15 being the highest you'll likely see) the stronger the signal.
			SerialUSB.println(rf95.lastRssi(), DEC);


			//* trigger one-shot mode pulse * // 20210312 
			trigger_one_shot_pulse();


			//digitalWrite(PIN_PULSEOUT_TO_CWU, HIGH);
			//delay(1);


			// Send a reply
	  //      uint8_t data[] = "And hello back to you";
	  //      rf95.send(data, sizeof(data));
	  //      rf95.waitPacketSent();
	  //      SerialUSB.println("Sent a reply");
			digitalWrite(PIN_LED, LOW);
		}
		else
		{

			SerialUSB.println("Receive failed");
		}
	}
}

// one-shot pulse generated
void trigger_one_shot_pulse()
{
	//if (!(REG_PORT_IN0 | ~PORT_PA16)) {// if (digitalRead(3) == LOW)

	if (TCC0->STATUS.bit.STOP)                                 // Check if the previous pulse is complete
	{
		TCC0->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;         // Retrigger the timer's One/Multi-Shot pulse
		while (TCC0->SYNCBUSY.bit.CTRLB);                        // Wait for synchronization
	}


}