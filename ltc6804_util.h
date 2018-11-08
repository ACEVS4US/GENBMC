/*! \file 
 * File:        ltc6804_util.h
 * Author:      Martin Clemons
 * Created:     August 5, 2013
 */
#pragma once

#include "ltc6804_regs.h"
//#include <sys/types.h>
#include <stdint.h>
#include <Arduino.h>



/***                Definitions                 ***/

/***          Public Global Variables           ***/

/***              Public Functions              ***/

void ltc6804util_initPec(void);
uint16_t ltc6804util_calcPec(uint8_t *data, int len);
double ltc6804util_convertSoc(uint16_t soc);
double ltc6804util_convertItmp(uint16_t itmp);
double ltc6804util_convertVa_Vd(uint16_t v);
double ltc6804util_convertV(uint16_t v);
void ltc6804util_encodeCommand(uint8_t addr, enum ltc6804_command_codes_e cmd, uint8_t *b);
void ltc6804util_decodeStatA(void *g, uint8_t *b);
void ltc6804util_decodeStatB(void *g, uint8_t *b);
void ltc6804util_decodeCfg(void *g, uint8_t *b);
void ltc6804util_encodeCfg(uint8_t *b, void *g);
void ltc6804util_decodeCV(void *g, uint8_t *b);
void ltc6804util_decodeAuxA(void *g, uint8_t *b);
void ltc6804util_decodeAuxB(void *g, uint8_t *b);

