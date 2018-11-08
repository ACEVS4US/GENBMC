/*ltc6804 driver* c++*/

#include "LTC6804.h"

LTC6804::LTC6804(){
  
  PIOB->PIO_PER = (1<<LTC6820_CS); //override peripheral on this pin
  PIOB->PIO_OER = (1<<LTC6820_CS); //set ddr of pin to output 

}

/*! Perform a dummy IO wake isoSPI devices.
 *  See '6820 datasheet fig. 15
 *   */
int LTC6804::wakeup(uint8_t bytes) {
  
  uint8_t b[bytes];          // buffer for dummy IO

  memset(b,0,bytes);
  ASSERT_SPI_CS;
  for(int i = 0;i<bytes;i++){
    SPI.transfer(b[i]);
  }
  RELEASE_SPI_CS;
  return 0;
  delayMicroseconds(310);  //datasheet max for Twake+tready to transition to ready
}


/*! Send a series of 2 byte commands with 2 byte PECs.
 *  \param fd SPI bus fd.
 *  \param address Array of addresses of device on isoSPI bus (LTC6804-2).
 *  \param cmd Array of commands to send.
 *  \param cnt Number of commands to send.
 *  \return returns 0 on success, -1 on failure.
 */
int LTC6804::sendCmd(uint8_t address, enum ltc6804_command_codes_e cmd) {
    
    //cannot have addresses greater than 16
    if (address > 0x0F) { 
      //fprintf_P(str, PSTR("sendCmd() with invalid 6804-2 address:%u\r\n"), (unsigned int)address);  
      Logger::info("sendCmd() with invalid 6804-2 address:%l",address);
        return -1;
    }
  
  //Ready the TX buffer for tranmission
    ltc6804util_encodeCommand(address, cmd, txB);

  //perform IO
    ASSERT_SPI_CS;
    for(int i = 0;i<4;i++){
      SPI.transfer(txB[i]);
    }

    for(int i=0;i<16;i++){
      rxB[i] = SPI.transfer(0x0);
      //Logger::info("sendCmd() received value:%l", rxB[i]);
    }
    RELEASE_SPI_CS;
    return 0;
}

/*! Perform an Address Write Command
 *  For more information see datasheet table 29.
 *  \param fd SPI bus fd.
 *  \param address Array of addresses of device on isoSPI bus (LTC6804-2).
 *  \param cmd Array of commands to send.
 *  \param encode Array of functions to encode register group structs.
 *  \param data_s Array of struct parameters for encode function.
 *  \param cnt Number of commands to send.
 *  \return returns 0 on success, -1 on failure.
 */
int LTC6804::addressWriteCmd(uint8_t address,enum ltc6804_command_codes_e cmd,void (*encode)(uint8_t *, void *),void *data_s){

    
     // 2 cmd bytes, 2 PEC bytes, 6 reg. group bytes, 2 PEC bytes required in buffer.
          if (encode == NULL) {
                Logger::info("addressWriteCmd() with NULL encode function.");
                return -1;
          }
          if (data_s == NULL) {
                Logger::info("addressWriteCmd() with NULL data_s parameter.");
                return -1;
          }
          if (address > 0x0F) {
                Logger::info("addressWriteCmd() with invalid 6804-2 address: %l", (unsigned int)address);
                return -1;
         }
         // put cmd and PEC in first 4 bytes of buffer
         ltc6804util_encodeCommand(address, cmd, txB);
       
         // put data and data PEC in 8 bytes following
         encode(&txB[4], data_s);   //execute function pointed to by encode
         // perform IO
         ASSERT_SPI_CS;
         for(int i=0;i<12;i++){
            SPI.transfer(txB[i]);
         }
         RELEASE_SPI_CS;

    return 0;
}

/*! Perform an Address Read Command
 *  For more information see datasheet table 31.
 *  \param fd SPI bus fd.
 *  \param address Array of addresses of device on isoSPI bus (LTC6804-2).
 *  \param cmd Array of commands to send.
 *  \param encode Array of functions to decode received register group structs.
 *  \param data_s Array of struct parameters for decode function.
 *  \param cnt Number of commands to send.
 *  \return returns 0 on success, -1 on failure.
 */
int LTC6804::addressReadCmd(uint8_t address,enum ltc6804_command_codes_e cmd,void (*decode)(void *, uint8_t *),void *data_s){

      // 2 cmd bytes, 2 PEC bytes, 6 reg. group bytes, 2 PEC bytes
      if (decode == NULL) {
              Logger::info("addressReadCmd() with NULL decode function.");
                 return -1;
      }
      if (data_s == NULL) {
           Logger::info("addressReadCmd() with NULL data_s parameter.");
                 return -1;
      }
      if (address > 0x0F) {
        Logger::info("addressReadCmd() with invalid 6804-2 address: %l", (unsigned int)address);
        return -1;
      }
      // assemble TX buffer
      memset(txB, 0, 16);
      
      //put cmd and PEC in first 4 bytes of current buffer offset
      ltc6804util_encodeCommand(address, cmd, txB);

      ASSERT_SPI_CS;
      //Send read command -4 bytes
      for(int i = 0;i < 4;i++){
          SPI.transfer(txB[i]);
      }

      //clear the rx buffer
      memset(rxB,0,16);
      //read the result into the rx buffer - the result in 6 bytes plus 2 pec bytes
      for (int i = 4;i<12;i++){
      rxB[i] = SPI.transfer(0x0);

      }
      
      RELEASE_SPI_CS;
      // decode data and data PEC in 8 bytes following 4 dummy command bytes.
      decode(data_s, &rxB[4]);
    return 0;
}



/***              Public Functions              ***/

/*! Initialize LTC6804
 *  \param address Address of LTC6804-2 IC.
 *  \param String with name of SPI interface. /dev/spidevx.y for example.
 *  \param SPI_speed SPI clock frequency in Hertz.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_init(uint8_t address) {
  
  
  
  //initialize the fast algorithm PEC Table
  ltc6804util_initPec();
    
    
    return 1;
}

void LTC6804::ltc6804_initIc(uint8_t address, struct ltc6804_cfg_reg_s *cfg)
{
  
  ltc6804util_initPec();
  
  //wakeup(1);                 // wake SPI interface IC, '6804.
  
  
  // start MUX diagnostics
  //ltc6804_startMuxDiag(address);
  //delay(5);     // wait for diagnostic to complete

  
  
    ltc6804_writeConfig(address, cfg,true);
    delay(5);  //trefup 4.4ms
}





/*! Close interface to LTC6804 devices
 */
void LTC6804::ltc6804_close(uint8_t address, struct ltc6804_cfg_reg_s *cfg) {
    
   wakeup(1);                 // wake SPI interface IC, '6804.
   
   // start MUX diagnostic
   //ltc6804_startMuxDiag(address);
   //delay(5);     // wait for diagnostic to complete
   // set configuration registers, GPIO pins
   
   ltc6804_writeConfig(address, cfg,false);

    
}

/*! Read LTC6804 status register group A.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_readStatA(uint8_t address, struct ltc6804_stat_reg_a_s *g,bool wake) {
    
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    
     // read register group
     g->pecOk = PEC_OK;
     addressReadCmd(address, CMD_RDSTATA, ltc6804util_decodeStatA, g);
   return 0;
}

/*! Read LTC6804 status register group B.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_readStatB(uint8_t address, struct ltc6804_stat_reg_b_s *g,bool wake) {
    
     // send dummy bit to wakeup '6820 and '6804,
     if(wake) wakeup(1);
     /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
     
     // read register group
     g->pecOk = PEC_OK;
     addressReadCmd(address, CMD_RDSTATB, ltc6804util_decodeStatB, g);
   return 0;
}

/*! Read LTC6804 configuration register group.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_readConfig(uint8_t address, struct ltc6804_cfg_reg_s *g,bool wake) {
    
  // send dummy bit to wakeup '6820 and '6804,
  if(wake) wakeup(1);
  /* Note: We assume here that the operations before next SPI io will
  * take at least 2 * t_ready and therefore we don't explicitly
  * insert a delay here.
  */
  
  // read register group
  g->pecOk = PEC_OK;
  addressReadCmd(address, CMD_RDCFG, ltc6804util_decodeCfg, g);

    
    return 0;
}

/*! Write LTC6804 configuration register group.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_writeConfig(uint8_t address, struct ltc6804_cfg_reg_s *g,bool wake) {
    
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    
    // write register group
    g->pecOk = PEC_OK;
    addressWriteCmd(address, CMD_WRCFG, ltc6804util_encodeCfg, g);

    return 0;
}

/*! Read LTC6804 cell voltage register group.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \param group Cell register group 'A' through 'D'.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_readCellVoltageGroup(uint8_t address, char group, struct ltc6804_cell_voltage_reg_s *g,bool wake) {
  
  // send dummy bit to wakeup '6820 and '6804,
  if(wake) wakeup(1);
  /* Note: We assume here that the operations before next SPI io will
   * take at least 2 * t_ready and therefore we don't explicitly
   * insert a delay here.
   */
   
  enum ltc6804_command_codes_e cmd;
  // set command
  switch (group) {
    case 'A':
         cmd = CMD_RDCVA;
         break;
    case 'B':
         cmd = CMD_RDCVB;
         break;
    case 'C':
         cmd = CMD_RDCVC;
         break;
    case 'D':
         cmd = CMD_RDCVD;
         break;
    default:
      Logger::info("ltc6804_readCellVoltageGroup(): cannot read from group %l", (int)group);
    return -1;
  }
  // read register group
  g->pecOk = PEC_OK;
  addressReadCmd(address, cmd, ltc6804util_decodeCV, g);
  return 0;
}

/*! Read all 4 LTC6804 cell voltage register group.
 *  \param address Address of LTC6804-2 IC.
 *  \param c Pointer to cell voltage struct into which data will be copied.
 *  Note: cell 1 voltage is stored in c.cell[0], cell 12 voltage in
 *  c.cell[11].
 *  \param group Cell register group 'A' through 'D'.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_readCellVoltageGroups(uint8_t address, struct ltc6804_cell_voltage_s *c,bool wake) {
    
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    
     
    enum ltc6804_command_codes_e cmd[] = {CMD_RDCVA, CMD_RDCVB, CMD_RDCVC, CMD_RDCVD};
    struct ltc6804_cell_voltage_reg_s g;
    int i;
    int j=0;
    c->pecOk = PEC_OK;
    for (i=0; i<4; i++) {
      g.pecOk = PEC_OK;
      // read register groups
      addressReadCmd(address, cmd[i], ltc6804util_decodeCV, &g);
      c->cell[j++] = g.offset[0];
      c->cell[j++] = g.offset[1];
      c->cell[j++] = g.offset[2];
      c->pecOk  = static_cast<ltc6804_PEC_e> (c->pecOk & g.pecOk);            // if any PEC failed, clear PEC.
    }

    
    return 0;
}

/*! Read LTC6804 auxiliary register group A.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_readAuxGroupA(uint8_t address, struct ltc6804_aux_reg_a_s *g,bool wake) {
    
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
    * take at least 2 * t_ready and therefore we don't explicitly
    * insert a delay here.
    */
    
    // read register group
    g->pecOk = PEC_OK;
    addressReadCmd(address, CMD_RDAUXA, ltc6804util_decodeAuxA, g);

    return 0;
}

/*! Read LTC6804 auxiliary register group B.
 *  \param address Address of LTC6804-2 IC.
 *  \param g Pointer to register group struct into which data will be copied.
 *  \return Function returns 0 on success, -1 on failure.
 */
int LTC6804::ltc6804_readAuxGroupB(uint8_t address, struct ltc6804_aux_reg_b_s *g,bool wake) {
    
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    
    // read register group
    g->pecOk = PEC_OK;
    addressReadCmd(address, CMD_RDAUXB, ltc6804util_decodeAuxB, g);
    
    return 0;
}

/*! Start ADC conversion of cell voltage
 *  \param address Address of LTC6804-2 IC.
 *  \param mode ADC mode (conversion speed)
 *  \param cells Cell mask indicating which cell voltages are converted.
 *  \return Function returns 0 on success, -1 on error.
 */
int LTC6804::ltc6804_startCellVoltageConversion(uint8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CH_e cells,bool wake) {
      
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    
    sendCmd(address, static_cast<ltc6804_command_codes_e> (CMD_ADCV | mode | cells));
    return 0;
}

/*! Start ADC conversion of GPIO voltage
 *  \param address Address of LTC6804-2 IC.
 *  \param mode ADC mode (conversion speed)
 *  \param chans Channel mask indicating which GPIO voltages are converted.
 *  \return Function returns 0 on success, -1 on error.
 */
int LTC6804::ltc6804_startGpioVoltageConversion(uint8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CHG_e chans,bool wake) {
    
  
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    
    sendCmd(address, static_cast<ltc6804_command_codes_e> (CMD_ADAX | mode | chans));

    return 0;
}

/*! Start ADC conversion of status group voltages
 *  \param address Address of LTC6804-2 IC.
 *  \param mode ADC mode (conversion speed).
 *  \param chans Channel mask indicating which status group voltages are converted.
 *  \return Function returns 0 on success, -1 on error.
 */
int LTC6804::ltc6804_startStatVoltageConversion(uint8_t address,
        enum ltc6804_ADC_modes_e mode,
        enum ltc6804_CHST_e chans,bool wake) {
     // send dummy bit to wakeup '6820 and '6804,
     if(wake) wakeup(1);
     /* Note: We assume here that the operations before next SPI io will
      * take at least 2 * t_ready and therefore we don't explicitly
      * insert a delay here.
      */
    
     sendCmd(address, static_cast<ltc6804_command_codes_e> (CMD_ADSTAT | mode | chans));

   
    return 0;
}

/*! Start MUX diagnostic.
 *  \param param address Address of LTC6804-2 IC.
 *  \return Function returns 0 on success, -1 on error.
 */
int LTC6804::ltc6804_startMuxDiag(uint8_t address,bool wake) {
    
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    
    sendCmd(address, CMD_DIAGN);
    return 0;
}

/*! Start ADC conversion of cell voltage and GPIO1, GPIO2
 *  \param address Address of LTC6804-2 IC.
 *  \param mode ADC mode (conversion speed).
 *  \return Function returns 0 on success, -1 on error.
 */
int LTC6804::ltc6804_startCellGpioVoltageConversion(uint8_t address,
        enum ltc6804_ADC_modes_e mode,bool wake) {
    
  
    // send dummy bit to wakeup '6820 and '6804,
    if(wake) wakeup(1);
    /* Note: We assume here that the operations before next SPI io will
     * take at least 2 * t_ready and therefore we don't explicitly
     * insert a delay here.
     */
    
    sendCmd(address,static_cast<ltc6804_command_codes_e> (CMD_ADCVAX | mode));

    
    return 0;
}
