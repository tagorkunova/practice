#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stdint.h>

/*
 * Minimal Modbus RTU library:
 * - Function 0x03 (Read Holding Registers)
 * - Function 0x06 (Write Single Register)
 * - Function 0x10 (Write Multiple Registers)
 */
int ModbusRTU_ProcessRequest(uint8_t slave_id,
                             const uint8_t *req,
                             uint16_t req_len,
                             uint16_t *holding,
                             uint16_t holding_count,
                             uint8_t *resp,
                             uint16_t *resp_len);

#endif
