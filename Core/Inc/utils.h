/*
 * utils.h
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include <stdint.h>

extern CRC_HandleTypeDef   hcrc;
extern TIM_HandleTypeDef htim4;

uint16_t util_crc16(const uint8_t* buf, uint32_t size);
uint16_t util_hw_crc16(uint8_t* buf, uint32_t size);
uint8_t crc_test(void);
void printBuffer(const uint8_t* buffer, uint32_t size);
#endif /* INC_UTILS_H_ */
