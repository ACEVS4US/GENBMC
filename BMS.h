//***************************************************************************
//
//  File........: BMS.h
//
//  Author(s)...: Chris Young
//
//  Target(s)...: ATSAM3X8ea-au
//
//  Compiler....: AVR-GCC 3.3.1; avr-libc 1.0
//
//  Description.: 
//
//  Revisions...: 1.0
//
//  YYYYMMDD - VER. - COMMENT                                       - SIGN.
//
//  20160502 - 1.0  - Created                                       - C.Young
//
//***************************************************************************

#ifndef BMS_H_
#define BMS_H_
#include "LTC6804.h"
#include "LTC6804_util.h"
#include "spi.h"
#include <Arduino.h>
#include "config.h"
#include "device.h"
#include "TickHandler.h"
#include "CanHandler.h"
#include "LTC6804.h"
#include <due_wire.h>

class BMSConfiguration : public DeviceConfiguration {
public:
  uint8_t num_slaves;
};

class BMS : public Device, CanObserver{
  
  public:
    BMS(void);
    DeviceType getType(void);
    DeviceId getId(void);
    void loadConfiguration(void);
    void saveConfiguration(void);
    void setup(void);
    void handleTick(void);
    void handleMessage(uint32_t messageType, void* message);
    void sendCmd(uint8_t cmd);
    void sendCanFrame1(void);
    void sendCanFrame2(void);
    void sendCanFrame3(void);

   

    //-- Enumerations --------------------------------------------------------------
    typedef enum bmsCommands{
          TRIGGER_MEASUREMENT = 0x1, // command: trigger a voltage measurement of all the cells.
          GET_TEMPERATURE = 0x2      // command: trigger a temperature measurement of all thermistors.
    };
    
  private:
    LTC6804* ltc6804;
    uint8_t num_of_slaves = 12;
    uint8_t addr=0;
    uint8_t cell_count = 0;
    uint8_t cell_start = 0;
    bool alarm_condition;
    uint8_t initialised = 0;
    uint16_t sum_as_int16 = 0;
    double sum = 0.0;
    double current_cell = 0.0;
    uint16_t min_cell_as_int16 = 0;
    double minimum_pack_cellV = 5.0;
    uint16_t max_cell_as_int16 = 65536;
    double maximum_pack_cellV = 0.0;
    uint16_t average_cell_as_int16 = 0;
    double average_pack_cellV = 0.0;
    double cell_voltages[96];
    double soc_lookup[101]={3.000, 3.084, 3.169, 3.245, 3.315, 3.340, 3.395, 3.438, 3.469, 3.483, 3.505, 3.522, 3.533, 3.544, 3.557, 3.563, 3.566, 3.569, 3.572, 3.582,
                            3.589, 3.592, 3.599, 3.605, 3.612, 3.618, 3.622, 3.627, 3.634, 3.639, 3.645, 3.651, 3.656, 3.661, 3.666, 3.670, 3.674, 3.678, 3.681, 3.686,
                            3.691, 3.698, 3.702, 3.706, 3.711, 3.715, 3.719, 3.723, 3.728, 3.733, 3.738, 3.744, 3.749, 3.756, 3.761, 3.768, 3.774, 3.779, 3.783, 3.789,
                            3.793, 3.797, 3.801, 3.805, 3.810, 3.814, 3.819, 3.823, 3.828, 3.832, 3.837, 3.841, 3.846, 3.850, 3.855, 3.860, 3.864, 3.869, 3.873, 3.878,
                            3.882, 3.886, 3.891, 3.895, 3.900, 3.905, 3.910, 3.915, 3.920, 3.925, 3.932, 3.940, 3.947, 3.955, 3.963, 3.971, 3.979, 3.988, 3.996, 4.053,
                            4.200};
    uint8_t low_cell_error_count = 0;
    uint8_t high_cell_error_count = 0;
    uint8_t low_cell_flag = 0;
    uint8_t high_cell_flag = 0;
    uint8_t general_error_flag = 0;
   
    double average = 0;
    uint8_t soc = 50;
    
    // set configuration registers, GPIO pins
    ltc6804_cfg_reg_s cfg = {
        .gpio = 0x01,                   // all GPIO pull-downs on except for pin 1
        .refon = 1,                     // leave reference on between conversions
        .swtrd = 0,                     // no effect
        .adcopt = 0,                    // ADC "standard" modes
        .vuv = 1874,                    // VUV = 3.0V
        .vov = 2625,                    // VOV 4.2V
        .dcc = 0,                       // no cell discharge
        .dcto = DCTO_DISABLED_gc       // disable DCTO
   };
    
    



};
#endif
