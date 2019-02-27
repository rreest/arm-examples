#include <stdio.h>
#include <stm32f3xx.h>


void GPIO_SPI_Init(GPIO_TypeDef *gpio, uint8_t pin, uint8_t);

void GPIO_Led_Init(GPIO_TypeDef *gpio, uint8_t pin);

/// Enable the SPI interface
void SPI_Enable();

void Blink_LED();

/*---------------------------------------------------------------------------*/

/* Fletcher 16 algorithm to calculate a checksum */
uint16_t fletcher16(const uint8_t *data, int count);

void spi_construct_msg(uint8_t cmd);

#define GPIO_AF6 6
#define L1_PIN 13

/* SPI data definitions*/
#define DATA_LEN 6
#define CRC_LEN 2
#define SPI_MSG_LEN (DATA_LEN + CRC_LEN)

int tx_index = 0;
int responding = 0;
/* For async command handling by the main loop, so stuff like led blinking doesnt hold up the IRQ. */
uint8_t cmd_to_handle = 0x00;

/* Divided by two because the MISO register is going to want 16 bits at a time.
 * If we want to send 8 bit bytes we'll need to aggregate our data beforehand. See below in spi_construct_msg */
uint16_t response[(SPI_MSG_LEN) / 2];

/*---------------------------------------------------------------------------*/

/// \brief SPI slave demo program on the STM32f302r8.
///
/// Runs the STM32 in slave mode, where it responds to a set protocol of commands.
/// Should be applicable to all of STM32f3xx (unless the pins have some special configuration
/// that no-one knows about).
///
/// Pinout (SPI3):
/// ---------------
/// A4  - SS
///	C10 - SCLK
///	C11 - MISO
///	C12 - MOSI
/// GND - GND -> Master's GND (Very important)
///
/// Communication:
/// ---------------
/// First the master should send a valid CMD byte. Then it should follow that with 11
/// more 0x00 bytes to get the full response. The first 4 bytes of the response
/// will be garbage and can be discarded. This is due to the STM32F3xx TX FIFO being
/// 32 bit long and demanding it be filled with data and not being able to be cleared.
/// By the time the slave receives the CMD byte, the TX is already full. The response
/// will be piped in after that. To get a synchronous response, I have not figured out
/// a different solution.
///
/// For an asynchronous solution, the master could send two messages - first the cmd,
/// and second the read bytes. The slave would have time to reset the SPI and pipe
/// in the right data from the get go. Again, this means the master won't get a response
/// immediately, and need more logic to figure out what was sent. Whether that's worth
/// saving 4 bytes is up to the designer.
///
/// After-note: actually if the slave needs a lot of time to process the cmd, then the
/// asynchronous solution is the only one possible. Synchronous is only viable if the
/// CMD is gonna take a few cycles max, all data needs to be prepared beforehand.
///
/// Speed:
/// ---------------
/// In testing up to 1MHz were sustainable. Speed could be improved by using DMA,
/// but I wanted to keep this example basic.
///
/// Response structure:
/// ---------------
/// | GB0 | GB1 | GB1 | GB2 | D0 | D1 | D2 | D3 | D4 | D5 | CRC-MSB | CRC-LSB |
///
/// .... where GBn stands for Garbage Byte N, Dn for Data byte N. CRC is for message
/// integrity checking. Data bytes may or may not contain anything meaningful.
///
/// CRC:
/// ---------------
/// CRC is done over data bytes 0-5.
/// SPI communication can be somewhat error-prone. To avoid accidentally reading in bad data,
/// a mechanism is used to check for errors in the transmission. A bad transmission can be
/// dropped and re-queried. The receiver should calculate one itself and check whether they match.
///
/// Commands:
/// ---------------
/// 0xf1 - ping. A simple ping.
/// Response: 0xfe
///
/// 0xf2 - blink led.
/// Response: 0xfe
///
/// 0xf3 - data query.
/// Response: 6x 8-byte numbers, 0 to 5.
///
/// Error codes:
/// ---------------
/// 0xfu - returned if the command is not recognized.
///
///
/// \return int. Exit status.
///
int main()
#pragma clang diagnostic ignored "-Wmissing-noreturn"
{
	/* Enable GPIOB, GPIOC, GPIOA */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;

	/* Configure Nucleo led 1 pin as output and high speed */
	GPIO_Led_Init(GPIOB, L1_PIN);

	GPIO_SPI_Init(GPIOA, 4, GPIO_AF6);
	GPIO_SPI_Init(GPIOC, 10, GPIO_AF6);
	GPIO_SPI_Init(GPIOC, 11, GPIO_AF6);
	GPIO_SPI_Init(GPIOC, 12, GPIO_AF6);

	// SPI configuration
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN; // Enable SPI on APB bus
	SPI3->CR1 &= ~SPI_CR1_MSTR; // Set as slave
	SPI3->CR2 |= 7 << SPI_CR2_DS_Pos; // Set to 8-bit transfers
	SPI3->CR2 |= SPI_CR2_FRXTH; // Generate RXNE event when fifo is at 8bit
	SPI3->CR2 |= SPI_CR2_TXEIE | SPI_CR2_RXNEIE; // Raise interrupt on TXE and RXNE
	NVIC_EnableIRQ(SPI3_IRQn); // Enable interrupts
	SPI_Enable();

	// Show that we're alive
	Blink_LED();

	while (1) {
		/* Reset half-finished transmit if the master has randomly dropped the connection. */
		if ((GPIOA->IDR & GPIO_ODR_4) && responding) {
			tx_index = 0;
			responding = 0;
		}
		if (cmd_to_handle == 0xf3) {
			Blink_LED();
			cmd_to_handle = 0x00;
		}
	}

	return 0;
}

void spi_construct_msg(uint8_t cmd)
{
	/* Response data. All 8 bit uints. */
	uint8_t rdata[DATA_LEN];
	int i;

	/* Create empty data. Make first byte 0xfa by default if we don't recognize the CMD. */
	rdata[0] = 0xfa;
	for (i = 1; i < DATA_LEN; i++) {
		rdata[i] = 0x00;
	}

	/* Handle the command. */
	if (cmd == 0xf1) {
		/* Ping */
		rdata[0] = 0xfe;
	} else if (cmd == 0xf2) {
		/* Blink led */
		rdata[0] = 0xfe;
		/* Tell the main loop to blink the led and continue with responding */
		cmd_to_handle = cmd;
	} else if (cmd == 0xf3) {
		/* Return numbers 0 to 5 */
		int z;
		for (z = 0; z < 6; z++) {
			rdata[z] = (uint8_t) z;
		}
	}

	/* Format the response according to protocol. */
	for (i = 0; i < DATA_LEN / 2; i++) {
		// Combine two bytes into one uint16 for SPI.
		response[i] = ((uint16_t) rdata[i * 2 + 1] << 8) | rdata[i * 2];
	}

	/* Make CRC */
	// Note that the SPI actually has a CRC module in it, but this is more
	// flexible as you get to choose the implementation, algorithm etc
	uint16_t crc = fletcher16(rdata, 6);
	/* Set CRC as the last 2 bytes of the response. */
	response[(SPI_MSG_LEN / 2) - 1] = crc;
}

void SPI3_IRQHandler()
{
	/* Check if the master has sent any data. */
	if (SPI3->SR & SPI_SR_RXNE) {
		uint8_t data; // The byte we received

		/* Read in the data. */
		data = (uint8_t) SPI3->DR;

		/* Ignore 0x00 as that's a read byte.
		 * Accept the command if we aren't responding to one. */
		if (data > 0x00 && responding == 0) {
			spi_construct_msg((uint8_t) data);
			/* Set the SPI into "responding to a cmd" mode. */
			responding = 1;
		}
	}

	/* Check if we are ready to send more data. */
	if (SPI3->SR & SPI_SR_TXE) {
		if (!responding) {
			/* Not responding to a cmd, just send dummy. */
			SPI3->DR = (uint16_t) 0xffff;
		} else {
			/* Send next 2 data bytes and increment counter so the next 2 are sent after that. */
			SPI3->DR = (uint16_t) response[tx_index++];

			/* Reset if the entire transmission has been completed. */
			if (tx_index >= SPI_MSG_LEN / 2) {
				/* Reset TX index. */
				tx_index = 0;
				/* Set not responding. */
				responding = 0;
			}
		}
	}
}


void SPI_Enable()
{
	SPI3->CR1 |= SPI_CR1_SPE;
}

void GPIO_Led_Init(GPIO_TypeDef *gpio, uint8_t pin)
{
	/* Set as output */
	gpio->MODER |= 0x01U << (pin * 2);

	/* Set to medium speed */
	gpio->OSPEEDR |= 0x02U << (pin * 2);
}

void GPIO_SPI_Init(GPIO_TypeDef *gpio, uint8_t pin, uint8_t af)
{
	/* Set to AF mode */
	gpio->MODER |= 0x2U << (pin * 2);

	/* Set high speed */
	gpio->OSPEEDR |= 0x3U << (pin * 2);

	/* Set AF. The ternaries toggle between high and low AF registers. Pins 0-7 use AFRL, 8-15 use AFRH. */
	gpio->AFR[pin > 8 ? 1 : 0] |= af << ((pin * 4) - (pin > 8 ? 32 : 0));
}

void Blink_LED()
{
	/* Turn on the led */
	GPIOB->BSRR |= (1 << L1_PIN);
	int i;
	for (i = 0; i < 100 * 10000; i++)
			asm("nop");

	GPIOB->BSRR |= (1 << (L1_PIN + 16));
}

/* Fletcher 16 algorithm to calculate a checksum */
uint16_t fletcher16(const uint8_t *data, int count)
{
	uint16_t sum1, sum2;
	int index;

	sum1 = 0;
	sum2 = 0;
	for (index = 0; index < count; ++index) {
		sum1 = (uint16_t) ((sum1 + data[index]) % 255);
		sum2 = (uint16_t) ((sum2 + sum1) % 255);
	}

	return (sum2 << 8) | sum1;
}
