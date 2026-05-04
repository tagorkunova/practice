#include "modbus_rtu.h"

static uint16_t mb_crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t j;

    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (j = 0; j < 8u; j++) {
            if ((crc & 0x0001u) != 0u) {
                crc >>= 1u;
                crc ^= 0xA001u;
            } else {
                crc >>= 1u;
            }
        }
    }
    return crc;
}

static void mb_put_u16_be(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)((value >> 8) & 0xFFu);
    dst[1] = (uint8_t)(value & 0xFFu);
}

static uint16_t mb_get_u16_be(const uint8_t *src) {
    return (uint16_t)(((uint16_t)src[0] << 8) | src[1]);
}

static void mb_append_crc(uint8_t *frame, uint16_t *len) {
    uint16_t crc = mb_crc16(frame, *len);
    frame[*len] = (uint8_t)(crc & 0xFFu);         /* CRC low */
    frame[*len + 1u] = (uint8_t)((crc >> 8) & 0xFFu); /* CRC high */
    *len += 2u;
}

static int mb_build_exception(uint8_t slave_id, uint8_t func, uint8_t code, uint8_t *resp, uint16_t *resp_len) {
    *resp_len = 0u;
    resp[0] = slave_id;
    resp[1] = (uint8_t)(func | 0x80u);
    resp[2] = code;
    *resp_len = 3u;
    mb_append_crc(resp, resp_len);
    return 1;
}

int ModbusRTU_ProcessRequest(uint8_t slave_id,
                             const uint8_t *req,
                             uint16_t req_len,
                             uint16_t *holding,
                             uint16_t holding_count,
                             uint8_t *resp,
                             uint16_t *resp_len) {
    uint8_t func;
    uint16_t crc_rx;
    uint16_t crc_calc;

    *resp_len = 0u;

    if (req_len < 4u) {
        return 0;
    }

    if (req[0] != slave_id) {
        return 0; /* Not our address */
    }

    crc_rx = (uint16_t)(((uint16_t)req[req_len - 1u] << 8) | req[req_len - 2u]);
    crc_calc = mb_crc16(req, (uint16_t)(req_len - 2u));
    if (crc_rx != crc_calc) {
        return 0;
    }

    func = req[1];

    if (func == 0x03u) {
        uint16_t start_addr;
        uint16_t quantity;
        uint16_t i;

        if (req_len != 8u) {
            return mb_build_exception(slave_id, func, 0x03u, resp, resp_len);
        }

        start_addr = mb_get_u16_be(&req[2]);
        quantity = mb_get_u16_be(&req[4]);

        if ((quantity == 0u) || (quantity > 125u)) {
            return mb_build_exception(slave_id, func, 0x03u, resp, resp_len);
        }
        if ((start_addr + quantity) > holding_count) {
            return mb_build_exception(slave_id, func, 0x02u, resp, resp_len);
        }

        resp[0] = slave_id;
        resp[1] = 0x03u;
        resp[2] = (uint8_t)(quantity * 2u);
        *resp_len = 3u;

        for (i = 0u; i < quantity; i++) {
            mb_put_u16_be(&resp[*resp_len], holding[start_addr + i]);
            *resp_len += 2u;
        }

        mb_append_crc(resp, resp_len);
        return 1;
    }

    if (func == 0x06u) {
        uint16_t addr;
        uint16_t value;

        if (req_len != 8u) {
            return mb_build_exception(slave_id, func, 0x03u, resp, resp_len);
        }

        addr = mb_get_u16_be(&req[2]);
        value = mb_get_u16_be(&req[4]);

        if (addr >= holding_count) {
            return mb_build_exception(slave_id, func, 0x02u, resp, resp_len);
        }

        holding[addr] = value;

        /* Echo request */
        resp[0] = slave_id;
        resp[1] = 0x06u;
        resp[2] = req[2];
        resp[3] = req[3];
        resp[4] = req[4];
        resp[5] = req[5];
        *resp_len = 6u;
        mb_append_crc(resp, resp_len);
        return 1;
    }

    if (func == 0x10u) {
        uint16_t start_addr;
        uint16_t quantity;
        uint8_t byte_count;
        uint16_t i;

        if (req_len < 9u) {
            return mb_build_exception(slave_id, func, 0x03u, resp, resp_len);
        }

        start_addr = mb_get_u16_be(&req[2]);
        quantity = mb_get_u16_be(&req[4]);
        byte_count = req[6];

        if ((quantity == 0u) || (quantity > 123u)) {
            return mb_build_exception(slave_id, func, 0x03u, resp, resp_len);
        }
        if (byte_count != (uint8_t)(quantity * 2u)) {
            return mb_build_exception(slave_id, func, 0x03u, resp, resp_len);
        }
        if (req_len != (uint16_t)(9u + byte_count)) {
            return mb_build_exception(slave_id, func, 0x03u, resp, resp_len);
        }
        if ((start_addr + quantity) > holding_count) {
            return mb_build_exception(slave_id, func, 0x02u, resp, resp_len);
        }

        for (i = 0u; i < quantity; i++) {
            holding[start_addr + i] = mb_get_u16_be(&req[7u + (uint16_t)(i * 2u)]);
        }

        resp[0] = slave_id;
        resp[1] = 0x10u;
        resp[2] = req[2];
        resp[3] = req[3];
        resp[4] = req[4];
        resp[5] = req[5];
        *resp_len = 6u;
        mb_append_crc(resp, resp_len);
        return 1;
    }

    return mb_build_exception(slave_id, func, 0x01u, resp, resp_len);
}
