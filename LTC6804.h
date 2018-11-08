/*! \file
   File:        ltc_6804.h
   Author:      martin
   Created:     July 27, 2013, 1:38 AM
*/
#pragma once

/***                Definitions                 ***/



#define LTC6820_CS 0
#define RELEASE_SPI_CS  PIOB -> PIO_SODR = (1<<LTC6820_CS)
#define ASSERT_SPI_CS  PIOB -> PIO_CODR = (1<<LTC6820_CS)



#include "ltc6804_regs.h"
#include <SPI.h>
#include "ltc6804_regs.h"
#include "ltc6804_util.h"
#include "Logger.h"

class LTC6804 {

  public:
    LTC6804(void);
    int ltc6804_init(uint8_t address);
    void ltc6804_initIc(uint8_t address, struct ltc6804_cfg_reg_s *cfg);
    void ltc6804_close(uint8_t address, struct ltc6804_cfg_reg_s *cfg);
    int ltc6804_readStatA(uint8_t address, struct ltc6804_stat_reg_a_s *g, bool wake);
    int ltc6804_readStatB(uint8_t address, struct ltc6804_stat_reg_b_s *g, bool wake);
    int ltc6804_readConfig(uint8_t address, struct ltc6804_cfg_reg_s *g, bool wake);
    int ltc6804_writeConfig(uint8_t address, struct ltc6804_cfg_reg_s *g, bool wake);
    int ltc6804_readCellVoltageGroup(uint8_t address, char group, struct ltc6804_cell_voltage_reg_s *g, bool wake);
    int ltc6804_readCellVoltageGroups(uint8_t address, struct ltc6804_cell_voltage_s *c, bool wake);
    int ltc6804_readAuxGroupA(uint8_t address, struct ltc6804_aux_reg_a_s *g, bool wake);
    int ltc6804_readAuxGroupB(uint8_t address, struct ltc6804_aux_reg_b_s *g, bool wake);
    int ltc6804_startCellVoltageConversion(uint8_t address,
                                           enum ltc6804_ADC_modes_e mode,
                                           enum ltc6804_CH_e cells, bool wake);
    int ltc6804_startGpioVoltageConversion(uint8_t address,
                                           enum ltc6804_ADC_modes_e mode,
                                           enum ltc6804_CHG_e chans, bool wake);
    int ltc6804_startStatVoltageConversion(uint8_t address,
                                           enum ltc6804_ADC_modes_e mode,
                                           enum ltc6804_CHST_e chans, bool wake);
    int ltc6804_startMuxDiag(uint8_t address, bool wake);
    int ltc6804_startCellGpioVoltageConversion(uint8_t address,
        enum ltc6804_ADC_modes_e mode, bool wake);

    /***              Private Functions              ***/
    /***        Private Function Prototypes         ***/
  private:
    int wakeup(uint8_t bytes);
    int sendCmd(uint8_t address, enum ltc6804_command_codes_e cmd);
    int addressWriteCmd(uint8_t address, enum ltc6804_command_codes_e cmd, void (*encode)(uint8_t *, void *), void *data_s);
    int addressReadCmd(uint8_t address, enum ltc6804_command_codes_e cmd, void (*decode)(void *, uint8_t *), void *data_s);


    static const int SPI_BUFFER_SIZE = 16;
    uint8_t txB[SPI_BUFFER_SIZE];
    uint8_t rxB[SPI_BUFFER_SIZE];

    FILE* str;

};
