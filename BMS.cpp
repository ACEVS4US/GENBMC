/*
 * BMS.cpp
 *
 * Created: 12/06/2016 11:08:46 a.m.
 *  Author: c.young
 */

#define NB_TARGET 1
#define OFFSET 0
#define BAUD 9600


 //_____ I N C L U D E S ________________________________________________________
#include "BMS.h"


 //create and initialise objects
 BMS::BMS(void){

   prefsHandler = new PrefHandler(LTC6804_BMS);
   commonName = "LTC6804 BMS";
   num_of_slaves = 12;
   ltc6804 = new LTC6804();
 }
 void BMS::setup(void)
 {
   TickHandler::getInstance()->detach(this);
   loadConfiguration();
   
   // register ourselves as observer of 0x31x can frames
   CanHandler::getInstanceEV()->attach(this, 0x310, 0x7f0, false);
   Logger::info("Attaching BMS to tick handler");
   TickHandler::getInstance()->attach(this, CFG_TICK_INTERVAL_BMS);
 }

 DeviceId BMS::getId() {
  return LTC6804_BMS;
 }

 DeviceType BMS::getType(){
  return (DEVICE_BMS);
}

 void BMS::handleMessage(uint32_t messageType, void* message){
  
 }
 
 
 void BMS::handleTick()
 {
   
   struct ltc6804_cell_voltage_s   cv[12];       // cell voltage register data
   struct ltc6804_stat_reg_a_s     sa[12];       // status register A group
   struct ltc6804_stat_reg_b_s     sb[12];       // status register B group
   struct ltc6804_aux_reg_a_s      xa[12];       // aux register A group
   
   //Logger::info("Number of BMS slaves at setup: %l",num_of_slaves);
   num_of_slaves=12;

   if(addr==12){
      sum = 0.0;
      minimum_pack_cellV = 5.0;
      maximum_pack_cellV = 0.0;
      for(int k = 0;k<96;k++){
        sum=sum+cell_voltages[k];
        if(cell_voltages[k]<minimum_pack_cellV) minimum_pack_cellV = cell_voltages[k];
        if(cell_voltages[k]>maximum_pack_cellV) maximum_pack_cellV = cell_voltages[k];   
      }
      average_pack_cellV = (double) sum/96;
      Logger::info("V Min Pack: %f  V Max Pack: %f  V Avg Pack: %f",minimum_pack_cellV,maximum_pack_cellV,average_pack_cellV);
      Serial.println();
      addr=0;

      
      max_cell_as_int16 = (uint16_t) (maximum_pack_cellV/100E-6);
      min_cell_as_int16 = (uint16_t) (minimum_pack_cellV/100E-6);
      average_cell_as_int16 = (uint16_t) (average_pack_cellV/100E-6);
      sum_as_int16 = (uint16_t) (sum/100E-6);

      //use the min cell to determine state of charge of the pack
      uint8_t h;
      for(h = 0;h<101;h++){
        if(minimum_pack_cellV<soc_lookup[h]){
          break;
        }
      }
      soc = h;
      //send can frame with this data
      sendCanFrame1();
      sendCanFrame2();

      if(minimum_pack_cellV<3.05){
        low_cell_error_count++;
      }
      else{
        if (low_cell_error_count >=1){
          low_cell_flag = 0;
          low_cell_error_count--;  // no persistent error so decrement the counter.
        }
      }
      if(low_cell_error_count>5){
        low_cell_flag = 1;
        sendCanFrame3();
      }

      if(maximum_pack_cellV>=4.18){
        high_cell_error_count++;
      }
      else{
        if(high_cell_error_count>=1){
          high_cell_flag = 0;
          high_cell_error_count--;
        }
      }
      if(high_cell_error_count>10){
        high_cell_flag=1;
        sendCanFrame3();
      }
      
      
   }
   
   if(initialised<12){
        Logger::debug("Initialising Slave: %l",addr);
        //set the configuration for this slave boards
        ltc6804->ltc6804_initIc(addr,&cfg);
        initialised++;    
   }

   //The watch dog timer expires after 2 seconds. This would reset the configuration registers, so we refresh them here to avoid this.
   //for(int j=0;j<12;j++){
   //    ltc6804->ltc6804_readConfig(addr,&cfg,true);
   //    ltc6804->ltc6804_writeConfig(addr,&cfg,true);
   //}
   
   cfg.gpio = 0x01;                   // all GPIO pull-downs on except for pin 1
   cfg.refon = 1;                     // leave reference on between conversions
   cfg.swtrd = 0;                     // read  only
   cfg.adcopt = 0;                    // ADC "standard" modes
   cfg.vuv = 0;                    // VUV = 3.0V
   cfg.vov = 0;                    // VOV 4.1V                       // no cell discharge
   cfg.dcc = 0; // cell to discharge
   cfg.dcto = DCTO_DISABLED_gc;                      // incase of comms errors after 1 minute quit the discharge

   //for each '6804 slave, assuming continuous address space
   //for (addr=3; addr<num_of_slaves; addr++) {
        
   if((addr == 1) ||(addr == 2)||(addr == 3)||(addr == 4) ||(addr == 6)||(addr == 7)||(addr == 8)||(addr == 9)||(addr == 10)||(addr == 11)){

   //ltc6804->ltc6804_initIc(addr,&cfg);
   

   //check all slaves are correctly configured
   ltc6804->ltc6804_readConfig(addr,&cfg,true); 
   Logger::console("IC Address: %l",addr);
   Logger::debug("CONFIG: GPIO %l  refon %l  swtrd %l  adcopt %l  vuv %l  vov %l  dcc %l,  dctc %l",cfg.gpio,cfg.refon,cfg.swtrd,cfg.adcopt,cfg.vuv,cfg.vov,cfg.dcc,cfg.dcto);
  
   // start all cell voltage groups and ADC conversions
   //at this point the core is likely in a sleep state
   ltc6804->ltc6804_startCellVoltageConversion(addr, ADC_MD_NORM_gc,CH_ALL_CELLS_gc,true);
   // wait for ADC to complete conversion
   delayMicroseconds(2480);  //to measure 12 cells in normal mode 
       
   //start status group ADC conversion
   //ltc6804->ltc6804_startStatVoltageConversion(addr, ADC_MD_NORM_gc, CHST_ALL_gc,false);

   // read cell voltage & aux registers
   ltc6804->ltc6804_readCellVoltageGroups(addr, &cv[addr],true);
   //ltc6804->ltc6804_readAuxGroupA(addr, &xa[addr]);

   /* we assume here that the time it takes to read the 4 cell voltage
    * register groups and the single auxA register group is sufficient for
    * the status ADC conversion to complete. This conversion takes 1.6ms
    * in the "NORMAL" ADC mode.
    */

    //ltc6804->ltc6804_readStatA(addr, &sa[addr],false);
    //ltc6804->ltc6804_readStatB(addr, &sb[addr],false);

    cell_count = 12;
    cell_start = 0; 
    if((addr==3)||(addr==4)||(addr==1)){
        cell_count = 8;
    }
    if((addr==10)){
        cell_count = 6;
    }
    if((addr==2)){
        cell_count = 10;
    }
    if(addr == 6){
        cell_start = 4; 
    }

    // output data to debug console 
   
    Serial.print("V cell: ");

    
    double current_cell = 0.0;
    double minimum_group_cell = 5.0;
    double maximum_group_cell = 0.0;
    uint8_t max_group_cell_index = 0;
    for (uint8_t cell=cell_start; cell<cell_count; cell++) {

        current_cell = cv[addr].cell[cell]*100E-6;
        //sum += current_cell;
        if(addr==1){
          cell_voltages[cell]=current_cell;
        }
        else if(addr == 2){
          cell_voltages[(cell+8)]=current_cell;
        }
        else if(addr == 3){
          cell_voltages[(cell+18)]=current_cell;
        }
        else if(addr == 4){
          cell_voltages[(cell+26)]=current_cell;
        }
        else if(addr == 6){
          cell_voltages[(cell+30)]=current_cell;
        }
        else if(addr == 7){
          cell_voltages[(cell+42)]=current_cell;
        }
        else if(addr == 8){
          cell_voltages[(cell+54)]=current_cell;
        }
        else if(addr == 9){
          cell_voltages[(cell+66)]=current_cell;
        }
        else if(addr == 10){
          cell_voltages[(cell+78)]=current_cell;
        }
        else if(addr == 11){
          cell_voltages[(cell+84)]=current_cell;
        }

        if(current_cell>maximum_group_cell){
          maximum_group_cell=current_cell;
          max_group_cell_index = cell;
        }

        //Print out all the cell voltages   
        Serial.print(current_cell,5);
        Serial.print(", ");
            
     }
     //average = sum/(cell_count-cell_start);
     Logger::info("Cell to discharge is: %l",max_group_cell_index);
     
     if (cv[addr].pecOk == PEC_OK) {
        Logger::debug("PEC ok");

        //only discharge if PEC okay
        //to save overheating only discharge 1 cell per board at once.
        //check if this cell needs discharging - it will need discharging if it is 0.015 volts above the minimum pack voltage
        if((maximum_group_cell>(minimum_pack_cellV+0.015))&&(maximum_group_cell>3.2)&&(initialised==12)){

               uint16_t discharge = 1<< max_group_cell_index;
               
               cfg.gpio = 0x01;                   // all GPIO pull-downs on except for pin 1
               cfg.refon = 1;                     // leave reference on between conversions
               cfg.swtrd = 0;                     // read  only
               cfg.adcopt = 0;                    // ADC "standard" modes
               cfg.vuv = 1874;                    // VUV = 3.0V
               cfg.vov = 2625;                    // VOV 4.2V                       // no cell discharge
               cfg.dcc = discharge; // cell to discharge
               cfg.dcto = DCTO_1_MIN_gc;                      // incase of comms errors after 1 minute quit the discharge 
               
               ltc6804->ltc6804_writeConfig(addr,&cfg,true);
               cfg.gpio = 0;                  
               cfg.refon = 0;                     
               cfg.swtrd = 0;                    
               cfg.adcopt = 0;                    
               cfg.vuv = 0;                   
               cfg.vov = 0;                    
               cfg.dcc = 0; 
               cfg.dcto = DCTO_DISABLED_gc;                      
              
               ltc6804->ltc6804_readConfig(addr,&cfg,true);
               Logger::debug("CONFIG: GPIO %l  refon %l  swtrd %l  adcopt %l  vuv %l  vov %l  dcc %l,  dctc %l",cfg.gpio,cfg.refon,cfg.swtrd,cfg.adcopt,cfg.vuv,cfg.vov,cfg.dcc,cfg.dcto);
  
         }
         else{
               cfg.gpio = 0x01;                   // all GPIO pull-downs on except for pin 1
               cfg.refon = 1;                     // leave reference on between conversions
               cfg.swtrd = 0;                     // no effect
               cfg.adcopt = 0;                    // ADC "standard" modes
               cfg.vuv = 1874;                    // VUV = 3.0V
               cfg.vov = 2625;                    // VOV 4.2V                       // no cell discharge
               cfg.dcc = 0; // no discharge
               cfg.dcto = DCTO_DISABLED_gc;       // disable DCTO
               
               ltc6804->ltc6804_writeConfig(addr,&cfg,true);
               cfg.gpio = 0;                   // all GPIO pull-downs on except for pin 1
               cfg.refon = 0;                     // leave reference on between conversions
               cfg.swtrd = 0;                     // no effect
               cfg.adcopt = 0;                    // ADC "standard" modes
               cfg.vuv = 0;                    // VUV = 3.0V
               cfg.vov = 0;                    // VOV 4.1V                       // no cell discharge
               cfg.dcc = 0; // cell to discharge
               cfg.dcto = DCTO_DISABLED_gc;                      // incase of comms errors after 1 minute quit the discharge 
               ltc6804->ltc6804_readConfig(addr,&cfg,true);
               Logger::debug("CONFIG: GPIO %l  refon %l  swtrd %l  adcopt %l  vuv %l  vov %l  dcc %l,  dctc %l",cfg.gpio,cfg.refon,cfg.swtrd,cfg.adcopt,cfg.vuv,cfg.vov,cfg.dcc,cfg.dcto);
  
         }
          
           
      }else {
           Logger::debug("PEC error");
      }
        
   }  
  
   
   addr++;
 }

void BMS::loadConfiguration() {
  BMSConfiguration *config = (BMSConfiguration *)getConfiguration();
  if(!config){
  config = new BMSConfiguration();
    setConfiguration(config);
  }
  Device::loadConfiguration(); // call parent

  if (prefsHandler->checksumValid()) { //checksum is good, read in the values stored in EEPROM
       prefsHandler->read(EESYS_BMS_NUM_DEVICES, &config->num_slaves);
       num_of_slaves= config->num_slaves;
       Logger::info("Number of BMS slaves at setup: %l",num_of_slaves);
  }
  else{ 
    config->num_slaves = 1;
    num_of_slaves=1;
    alarm_condition = true;
  }
}

void BMS::saveConfiguration() {
  BMSConfiguration *config = (BMSConfiguration *) getConfiguration();
  Device::saveConfiguration(); // call parent
  config->num_slaves= (uint8_t) num_of_slaves;
  prefsHandler->write(EESYS_BMS_NUM_DEVICES, config->num_slaves);
  prefsHandler->saveChecksum();
  Logger::info("Number of BMS slaves at save: %l",config->num_slaves);
  loadConfiguration();
}

void BMS::sendCmd(uint8_t cmd){
  uint8_t command = cmd;
  Logger::info("Command sent: %l",command);
  switch(command){
    case TRIGGER_MEASUREMENT:
       
    break;
    case GET_TEMPERATURE:
       
    break;
  }
  
}

void BMS::sendCanFrame1(void){
  CAN_FRAME output;
  output.length = 8;  //number of frames to send
  output.id = 0x301;
  output.extended = 0; //standard frame
  output.rtr = 0;
  
  output.data.bytes[0] = (uint8_t) (min_cell_as_int16>>8);  //msbs
  output.data.bytes[1] = (uint8_t) min_cell_as_int16; //lsbs

  output.data.bytes[2] = (uint8_t) (max_cell_as_int16>>8);  //msbs
  output.data.bytes[3] = (uint8_t) max_cell_as_int16; //lsbs

  output.data.bytes[4] = (uint8_t) (average_cell_as_int16>>8);  //msbs
  output.data.bytes[5] = (uint8_t) average_cell_as_int16; //lsbs

  output.data.bytes[6] = 0;  //msbs
  output.data.bytes[7] = 0; //lsbs

  
  CanHandler::getInstanceEV()->sendFrame(output);

  
}
 
 void BMS::sendCanFrame2(void){
  CAN_FRAME output;
  output.id = 0x302;
  output.length = 8;
  output.extended = 0;
  output.rtr = 0;

  //pack voltage
  output.data.bytes[0] = (uint8_t) (sum_as_int16>>8);  //msbs
  output.data.bytes[1] = (uint8_t) sum_as_int16; //lsbs

  //Pack current
  output.data.bytes[2] = 0;  //msbs
  output.data.bytes[3] = 0; //lsbs

  //SOC
  output.data.bytes[4] = soc;  //msbs
  CanHandler::getInstanceEV()->sendFrame(output);
 }

 void BMS::sendCanFrame3(void){
  CAN_FRAME output;
  output.id = 0x303;
  output.length = 1;
  output.extended = 0;
  output.rtr = 0;

  //error flags
  output.data.bytes[0] = (general_error_flag)|(low_cell_flag<<2)|(high_cell_flag<<3); //general error
  CanHandler::getInstanceEV()->sendFrame(output);


 }

 

