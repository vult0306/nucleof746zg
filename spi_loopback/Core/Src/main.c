#include "main.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

uint8_t tx_buf[] = "spi loopback\r\n";  // string for tx spi
uint8_t tx_len = 0;
uint8_t rx_buf[100];                    // buffer for rx spi
uint8_t rx_len = 0;

void rx_callback();                     // spi RX interrupt callback
static void MX_SPI1_Init(void);         // initialize SPI module

int main(void)
{
    // init system
    System_init();

    // init spi module
    MX_SPI1_Init();

    while (1)
    {
        // sending string
        if (tx_len < sizeof(tx_buf)) {
            while((SPI1->SR & SPI_SR_TXE) == 0){};          // wait until TX register is empty
            SPI1->DR = (uint16_t)(tx_buf[tx_len++]);        // transmit data
        }

        // print out received string
        if (rx_len == sizeof(tx_buf)) {
            printf((char*)rx_buf);
            memset(rx_buf, 0, sizeof(rx_buf));
            rx_len = 0;
        }
    }
}

static void MX_SPI1_Init(void)
{
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* make sure spi is disabled */
    SPI1->CR1 &= ~SPI_CR1_SPE;


    /*----------------------- SPIx CR1 & CR2 Configuration ---------------------*/
    /* Configure : SPI Mode, Communication Mode, Clock polarity and phase, NSS management,
    Communication speed, First bit and CRC calculation state */
    SPI1->CR1 |= SPI_MODE_MASTER |
                 SPI_DIRECTION_2LINES |
                 SPI_POLARITY_LOW |
                 SPI_PHASE_1EDGE |
                 SPI_NSS_SOFT |
                 SPI_BAUDRATEPRESCALER_2 |
                 SPI_FIRSTBIT_MSB |
                 SPI_CRCCALCULATION_DISABLE;

    /* Configure : NSS management, TI Mode, NSS Pulse, Data size and Rx Fifo threshold */
    SPI1->CR2 |= ((SPI_NSS_SOFT >> 16) & SPI_CR2_SSOE) |
                   SPI_TIMODE_DISABLE |
                   SPI_NSS_PULSE_ENABLE |
                   SPI_DATASIZE_8BIT |
                   SPI_RXFIFO_THRESHOLD_HF ;

    /* Set priority for SPI1_IRQn */
    NVIC_SetPriority(SPI1_IRQn, 0);
    /* Enable SPI1_IRQn           */
    NVIC_EnableIRQ(SPI1_IRQn);

    /* enable received interrupt */
    SPI1->CR2 |= SPI_CR2_RXNEIE;

    /* enable module SPI */
    SPI1->CR1 |= SPI_CR1_SPE;

}

// SPI received callback function
void rx_callback() {
    if (rx_len < sizeof(tx_buf)){
        rx_buf[rx_len++] = (uint8_t)(SPI1->DR);     // read data from data register
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
