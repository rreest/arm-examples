#include <stdio.h>
#include <stm32f3xx.h>

#define L1_PIN 13
#define GPIO_AF6 6
#define MAX_BOOT_BLINK_COUNT 3

int boot_blinks = 0;

void Blink_LED();

void SPI_Send(uint16_t);

uint16_t SPI_Disable();

void SPI_Enable();

/// \brief SPI slave demo program on the STM32f302r8.
///
/// Should be applicable to all of STM32f3xx (unless the pins have some special configuration
/// that no-one knows about).
/// \return int. Exit status.
///
int main()
#pragma clang diagnostic ignored "-Wmissing-noreturn"
{
	return 0;
}
