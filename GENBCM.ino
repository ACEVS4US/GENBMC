/*
 GEVCU.ino
 
 Created: 1/4/2013 1:34:14 PM
 Author: Collin Kidder
 
 New, new plan: Allow for an arbitrary # of devices that can have both tick and canbus handlers. These devices register themselves
 into the handler framework and specify which sort of device they are. They can have custom tick intervals and custom can filters.
 The system automatically manages when to call the tick handlers and automatically filters canbus and sends frames to the devices.
 There is a facility to send data between devices by targetting a certain type of device. For instance, a motor controller
 can ask for any throttles and then retrieve the current throttle position from them.

Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */
 
/*Changelog removed. All changes are logged to GIT */

/*
Random comments on things that should be coded up soon:
4. It is a possibility that there should be support for actually controlling the power to some of the devices.
	For instance, power could be controlled to the +12V connection at the DMOC so that it can be power cycled
	in software. But, that uses up an input and people can just cycle the key (though that resets the GEVCU too)
5. Some people (like me, Collin) have a terrible habit of mixing several coding styles. It would be beneficial to
	continue to harmonize the source code - Perhaps use a tool to do this.
6. It should be possible to limit speed and/or torque in reverse so someone doesn't kill themselves or someone else
	while gunning it in reverse - The configuration variable is there and settable now. Just need to integrate it.
7. The DMOC code duplicates a bunch of functionality that the base class also used to implement. We've got to figure
	out where the overlaps are and fix it up so that as much as possible is done generically at the base MotorController
	class and not directly in the Dmoc class.
*/

#include "GENBCM.h"

// The following includes are required in the .ino file by the Arduino IDE in order to properly
// identify the required libraries for the build.
#include <due_rtc.h>
#include <due_can.h>
#include <due_wire.h>
#include <DueTimer.h>

//RTC_clock rtc_clock(XTAL); //init RTC with the external 32k crystal as a reference

//Evil, global variables
CanHandler *canHandlerEV;
CanHandler *canHandlerCar;
TickHandler *tickHandler;
PrefHandler *sysPrefs;
MemCache *memCache;
Heartbeat *heartbeat;
SerialConsole *serialConsole;
//Device *wifiDevice;
//Device *btDevice;


byte i = 0;


void sendWiReach(char* message)
{
  /*
  Serial2.println(message);
    delay(700);
    while (Serial2.available()) {SerialUSB.write(Serial2.read());}
    */
}

void initWiReach()
{
 
//Serial.begin(9600); // use SerialUSB only as the programming port doesn't work
//Serial2.begin(115200); // use Serial3 for GEVCU2, use Serial2 for GEVCU3+4
/*
sendWiReach("AT+iFD");//Host connection set to serial port
delay(5000);
sendWiReach("AT+iHIF=1");//Host connection set to serial port
sendWiReach("AT+iBDRF=9");//Automatic baud rate on host serial port
sendWiReach("AT+iRPG=secret"); //Password for iChip wbsite
sendWiReach("AT+iWPWD=secret");//Password for our website
sendWiReach("AT+iWST0=0");//Connection security wap/wep/wap2 to no security
sendWiReach("AT+iWLCH=4");  //Wireless channel
sendWiReach("AT+iWLSI=GEVCU");//SSID
sendWiReach("AT+iWSEC=1");//IF security is used, set for WPA2-AES
sendWiReach("AT+iSTAP=1");//Act as AP
sendWiReach("AT+iDIP=192.168.3.10");//default ip - must be 10.x.x.x
sendWiReach("AT+iDPSZ=8");//DHCP pool size
sendWiReach("AT+iAWS=1");//Website on
sendWiReach("AT+iDOWN");//Powercycle reset
delay(5000);
SerialUSB.println("WiReach Wireless Module Initialized....");
*/
}


  

//initializes all the system EEPROM values. Chances are this should be broken out a bit but
//there is only one checksum check for all of them so it's simple to do it all here.

void initSysEEPROM() {
	//three temporary storage places to make saving to EEPROM easy
	uint8_t eight;
	uint16_t sixteen;
	uint32_t thirtytwo;

	eight = 4; //GEVCU4 or GEVCU5 boards
	sysPrefs->write(EESYS_SYSTEM_TYPE, eight);

	sixteen = 1024; //no gain
	sysPrefs->write(EESYS_ADC0_GAIN, sixteen);
	sysPrefs->write(EESYS_ADC1_GAIN, sixteen);
	sysPrefs->write(EESYS_ADC2_GAIN, sixteen);
	sysPrefs->write(EESYS_ADC3_GAIN, sixteen);

	sixteen = 0; //no offset
	sysPrefs->write(EESYS_ADC0_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC1_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC2_OFFSET, sixteen);
	sysPrefs->write(EESYS_ADC3_OFFSET, sixteen);

	sixteen = 500; //multiplied by 1000 so 500k baud
	sysPrefs->write(EESYS_CAN0_BAUD, sixteen);
	sysPrefs->write(EESYS_CAN1_BAUD, sixteen);

	sixteen = 11520; //multiplied by 10
	sysPrefs->write(EESYS_SERUSB_BAUD, sixteen);

	sixteen = 100; //multiplied by 1000
	sysPrefs->write(EESYS_TWI_BAUD, sixteen);

	sixteen = 100; //number of ticks per second
	sysPrefs->write(EESYS_TICK_RATE, sixteen);

	thirtytwo = 0;
	sysPrefs->write(EESYS_RTC_TIME, thirtytwo);
	sysPrefs->write(EESYS_RTC_DATE, thirtytwo);

	eight = 5; //how many RX mailboxes
	sysPrefs->write(EESYS_CAN_RX_COUNT, eight);

	thirtytwo = 0x7f0; //standard frame, ignore bottom 4 bits
	sysPrefs->write(EESYS_CAN_MASK0, thirtytwo);
	sysPrefs->write(EESYS_CAN_MASK1, thirtytwo);
	sysPrefs->write(EESYS_CAN_MASK2, thirtytwo);
	sysPrefs->write(EESYS_CAN_MASK3, thirtytwo);
	sysPrefs->write(EESYS_CAN_MASK4, thirtytwo);

	thirtytwo = 0x230;
	sysPrefs->write(EESYS_CAN_FILTER0, thirtytwo);
	sysPrefs->write(EESYS_CAN_FILTER1, thirtytwo);
	sysPrefs->write(EESYS_CAN_FILTER2, thirtytwo);

	thirtytwo = 0x650;
	sysPrefs->write(EESYS_CAN_FILTER3, thirtytwo);
	sysPrefs->write(EESYS_CAN_FILTER4, thirtytwo);

	thirtytwo = 0; //ok, not technically 32 bytes but the four zeros still shows it is unused.
	sysPrefs->write(EESYS_WIFI0_SSID, thirtytwo);
	sysPrefs->write(EESYS_WIFI1_SSID, thirtytwo);
	sysPrefs->write(EESYS_WIFI2_SSID, thirtytwo);
	sysPrefs->write(EESYS_WIFIX_SSID, thirtytwo);

	eight = 0; //no channel, DHCP off, B mode
	sysPrefs->write(EESYS_WIFI0_CHAN, eight);
	sysPrefs->write(EESYS_WIFI0_DHCP, eight);
	sysPrefs->write(EESYS_WIFI0_MODE, eight);

	sysPrefs->write(EESYS_WIFI1_CHAN, eight);
	sysPrefs->write(EESYS_WIFI1_DHCP, eight);
	sysPrefs->write(EESYS_WIFI1_MODE, eight);

	sysPrefs->write(EESYS_WIFI2_CHAN, eight);
	sysPrefs->write(EESYS_WIFI2_DHCP, eight);
	sysPrefs->write(EESYS_WIFI2_MODE, eight);

	sysPrefs->write(EESYS_WIFIX_CHAN, eight);
	sysPrefs->write(EESYS_WIFIX_DHCP, eight);
	sysPrefs->write(EESYS_WIFIX_MODE, eight);

	thirtytwo = 0;
	sysPrefs->write(EESYS_WIFI0_IPADDR, thirtytwo);
	sysPrefs->write(EESYS_WIFI1_IPADDR, thirtytwo);
	sysPrefs->write(EESYS_WIFI2_IPADDR, thirtytwo);
	sysPrefs->write(EESYS_WIFIX_IPADDR, thirtytwo);

	sysPrefs->write(EESYS_WIFI0_KEY, thirtytwo);
	sysPrefs->write(EESYS_WIFI1_KEY, thirtytwo);
	sysPrefs->write(EESYS_WIFI2_KEY, thirtytwo);
	sysPrefs->write(EESYS_WIFIX_KEY, thirtytwo);

	eight = 1;
	sysPrefs->write(EESYS_LOG_LEVEL, eight);

	sysPrefs->saveChecksum();
}

void createObjects() {
	BMS *bms = new BMS();
  bms->setup();  //this only need to be here while the epprom isn't working
}

void initializeDevices() {
	DeviceManager *deviceManager = DeviceManager::getInstance();

	//heartbeat is always enabled now
	heartbeat = new Heartbeat();
	Logger::info("add: Heartbeat (id: %X, %X)", HEARTBEAT, heartbeat);
	heartbeat->setup();

	/*
	We used to instantiate all the objects here along with other code. To simplify things this is done somewhat
	automatically now. Just instantiate your new device object in createObjects above. This takes care of the details
	so long as you follow the template of how other devices were coded.
	*/
	createObjects(); 

	/*
	 *	We defer setting up the devices until here. This allows all objects to be instantiated
	 *	before any of them set up. That in turn allows the devices to inspect what else is
	 *	out there as they initialize. For instance, a motor controller could see if a BMS
	 *	exists and supports a function that the motor controller wants to access.
	 */
	deviceManager->sendMessage(DEVICE_ANY, INVALID, MSG_STARTUP, NULL);

}

void setup() {
  delay(500);  //This delay lets you see startup.      
  //Serial.begin(CFG_SERIAL_SPEED);
  Serial.begin(9600);
  Serial.println(CFG_VERSION);
  Serial.print("Build number: ");
  Serial.println(CFG_BUILD_NUM);     
       // initWiReach();
	pinMode(2, OUTPUT);
	digitalWrite(2, HIGH);
  delay(1000);  //This delay lets you see startup.  
  digitalWrite(2, LOW);
  delay(1000);
  
	Wire.begin();
  Logger::info("TWI init ok");
  // start the SPI library:
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);

	
	memCache = new MemCache();
	Logger::info("add MemCache (id: %X, %X)", MEMCACHE, memCache);
	memCache->setup();
	sysPrefs = new PrefHandler(SYSTEM);
	if (!sysPrefs->checksumValid()) 
     {
	      Logger::info("Initializing EEPROM");
	      initSysEEPROM();
        initWiReach();
	   } 
        else {Logger::info("Using existing EEPROM values");}//checksum is good, read in the values stored in EEPROM

	uint8_t loglevel;
	sysPrefs->read(EESYS_LOG_LEVEL, &loglevel);
	Logger::setLoglevel((Logger::LogLevel)loglevel);
	//Logger::setLoglevel((Logger::LogLevel)1);
	sys_early_setup();     
  tickHandler = TickHandler::getInstance();
  canHandlerEV = CanHandler::getInstanceEV();
  canHandlerCar = CanHandler::getInstanceCar();
  canHandlerEV->initialize();
  canHandlerCar->initialize();
	
	setup_sys_io(); //get calibration data for system IO
	Logger::info("SYSIO init ok");

	initializeDevices();
    serialConsole = new SerialConsole(memCache, heartbeat);
	serialConsole->printMenu();
	//wifiDevice = DeviceManager::getInstance()->getDeviceByID(ICHIP2128);
//	btDevice = DeviceManager::getInstance()->getDeviceByID(ELM327EMU);
    //DeviceManager::getInstance()->sendMessage(DEVICE_WIFI, ICHIP2128, MSG_CONFIG_CHANGE, NULL); //Load configuration variables into WiFi Web Configuration screen
	Logger::info("System Ready");
}

void loop() {

//#ifdef CFG_TIMER_USE_QUEUING
	tickHandler->process();
//#endif
  canHandlerEV->process();
  canHandlerCar->process();
	serialConsole->loop();
	//TODO: this is dumb... shouldn't have to manually do this. Devices should be able to register loop functions
	//if ( wifiDevice != NULL ) {
	//	((ICHIPWIFI*)wifiDevice)->loop();
	//}


	//this should still be here. It checks for a flag set during an interrupt
	//sys_io_adc_poll();
}


