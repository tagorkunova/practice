#include "stm32f10x.h"
#include <stdint.h>
#include <string.h>

#include "modbus_rtu.h"

#define MODBUS_SLAVE_ID  1u
#define MODBUS_BAUD_BRR  0x345u /* 8 MHz APB2, 9600 baud */
#define UART_ECHO_TEST   0u     /* 1: raw UART echo test, 0: Modbus RTU */
#define MB_FRAME_INCOMPLETE 0u
#define MB_FRAME_INVALID    0xFFFFu

static void gpio_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    /* PA0: input with pull-up (button to GND) */
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;
    GPIOA->ODR |= GPIO_ODR_ODR0;

    /* PC13: onboard LED output */
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;

    GPIOC->BSRR = GPIO_BSRR_BS13; /* LED off for active-low wiring */
}

static void usart1_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9 (TX): AF push-pull, 50 MHz */
    GPIOA->CRH &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    GPIOA->CRH |= GPIO_CRH_MODE9 | GPIO_CRH_CNF9_1;

    /* PA10 (RX): input floating */
    GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_CNF10_0;

    USART1->BRR = MODBUS_BAUD_BRR;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

static void usart1_send_bytes(const uint8_t *data, uint16_t len) {
    uint16_t i;
    for (i = 0; i < len; i++) {
        while ((USART1->SR & USART_SR_TXE) == 0) {
        }
        USART1->DR = data[i];
    }
    while ((USART1->SR & USART_SR_TC) == 0) {
    }
}

static int usart1_try_read_byte(uint8_t *b) {
    if ((USART1->SR & USART_SR_RXNE) != 0) {
        *b = (uint8_t)(USART1->DR & 0xFFu);
        return 1;
    }
    return 0;
}

static void led_apply(uint16_t value) {
    if (value != 0u) {
        GPIOC->BRR = GPIO_BRR_BR13;   /* on */
    } else {
        GPIOC->BSRR = GPIO_BSRR_BS13; /* off */
    }
}

static uint16_t modbus_expected_req_len(const uint8_t *buf, uint16_t len) {
    if (len < 2u) {
        return MB_FRAME_INCOMPLETE;
    }

    if ((buf[1] == 0x03u) || (buf[1] == 0x06u)) {
        return 8u;
    }

    if (buf[1] == 0x10u) {
        if (len < 7u) {
            return MB_FRAME_INCOMPLETE;
        }
        if (buf[6] > 246u) {
            return MB_FRAME_INVALID;
        }
        return (uint16_t)(9u + (uint16_t)buf[6]);
    }

    return MB_FRAME_INVALID;
}

#if UART_ECHO_TEST
int main(void) {
    static const uint8_t banner[] = "UART1 echo ready\r\n";
    uint32_t heartbeat = 0u;
    uint16_t led_state = 0u;

    gpio_init();
    usart1_init();
    usart1_send_bytes(banner, (uint16_t)(sizeof(banner) - 1u));

    while (1) {
        uint8_t b;

        heartbeat++;
        if (heartbeat >= 2000000u) {
            heartbeat = 0u;
            led_state = (uint16_t)(1u - led_state);
            led_apply(led_state); /* periodic heartbeat: proves firmware is running */
        }

        if (usart1_try_read_byte(&b)) {
            if (b == (uint8_t)'1') {
                led_apply(1u);
                led_state = 1u;
            } else if (b == (uint8_t)'0') {
                led_apply(0u);
                led_state = 0u;
            }
            usart1_send_bytes(&b, 1u); /* Echo every received byte */
        }
    }
}
#else
int main(void) {
    uint16_t holding[8] = {0};
    uint8_t rx_buf[256];
    uint8_t tx_buf[256];
    uint16_t rx_len = 0;
    uint16_t expected_len = 0;
    uint32_t rx_idle = 0;

    gpio_init();
    usart1_init();

    while (1) {
        uint8_t b;
        uint16_t read_count = 0;

        while (usart1_try_read_byte(&b)) {
            read_count++;
            if (rx_len < (uint16_t)sizeof(rx_buf)) {
                rx_buf[rx_len++] = b;
            } else {
                rx_len = 0;
            }
        }

        if (read_count != 0u) {
            rx_idle = 0u;
        } else if (rx_len != 0u) {
            rx_idle++;
            if (rx_idle > 120000u) {
                rx_len = 0;
                rx_idle = 0u;
            }
        }

        /* Keep register map in sync with hardware */
        holding[1] = ((GPIOA->IDR & GPIO_IDR_IDR0) == 0u) ? 1u : 0u; /* button */
        led_apply(holding[0]); /* LED command from Modbus register 0 */

        expected_len = modbus_expected_req_len(rx_buf, rx_len);
        if (expected_len == MB_FRAME_INVALID) {
            if (rx_len > 1u) {
                memmove(rx_buf, &rx_buf[1], (size_t)(rx_len - 1u));
                rx_len--;
            } else {
                rx_len = 0;
            }
        } else if ((expected_len != MB_FRAME_INCOMPLETE) && (expected_len <= (uint16_t)sizeof(rx_buf)) && (rx_len >= expected_len)) {
            uint16_t tx_len = 0;
            if (ModbusRTU_ProcessRequest(MODBUS_SLAVE_ID, rx_buf, expected_len, holding, 8u, tx_buf, &tx_len) && (tx_len > 0u)) {
                usart1_send_bytes(tx_buf, tx_len);
            }
            if (rx_len > expected_len) {
                memmove(rx_buf, &rx_buf[expected_len], (size_t)(rx_len - expected_len));
            }
            rx_len = (uint16_t)(rx_len - expected_len);
            rx_idle = 0u;
        } else if ((expected_len != MB_FRAME_INCOMPLETE) && (expected_len > (uint16_t)sizeof(rx_buf))) {
            rx_len = 0;
            rx_idle = 0u;
        } else if (rx_len >= (uint16_t)sizeof(rx_buf)) {
            rx_len = 0;
            rx_idle = 0u;
        }
    }
}
#endif
