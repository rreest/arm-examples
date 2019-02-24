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

/// \brief SPI master demo program on the STM32f302r8.
///
/// Should be applicable to all of STM32f3xx (unless the pins have some special configuration
/// that no-one knows about).
/// \return int. Exit status.
///
int main()
#pragma clang diagnostic ignored "-Wmissing-noreturn"
{
	/* Enable GPIOB, GPIOC, GPIOA */
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN;
	/* Configure Nucleo led 1 pin as output and high speed */
	GPIOB->MODER |= GPIO_MODER_MODER13_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13_1;

	/* Enable Timer 2
	 *
	 * - Enable timer clock in APB1 bus
	 * - Turn on CEN bit to enable counter
	 * - Set pre-scaler (PSC) to X to tick the timer APB1_CLOCK/x times per second
	 * - Set the timer to count to Y (ARR)
	 * - Set the timer to raise an interrupt when it has clocked out (DIER)
	 * - Enable NVIC interrupt (to call handler TIM2_IRQHandler)
	 */
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->CR1 |= TIM_CR1_CEN;
	TIM2->PSC |= (uint16_t) 4875;
	TIM2->ARR = (uint16_t) 5000;
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable and configure SPI2
	 *
	 * NOTE: Anything that is commented out in the spi-master below is a setting
	 * that is already set to that value by default.
	 * It may need to be changed in a specific configuration,
	 * or reset by hand to make sure it has the correct value,
	 * but most things are already set correctly at reset.
	 *
	 * - Enable GPIO A and C (PA4 and PC10, 11, 12 used)
	 * - Configure GPIO-s
	 *    - Set GPIO to AF (Alternate function) mode
	 *    - Set GPIO to high speed
	 *    - (Optional) Set output type (already set by default)
	 *    - (Optional) Set GPIO as no pullup/down (already set by default)
	 *    - Set GPIO to SPI alternate function
	 * - Enable SPI clock on the APB1 bus
	 * - Set SPI baud rate
	 * - Set SPI to master mode
	 * - Set SPI packet size (between 4 and 16 bit)
	 * - Tell SPI when to generate RXNE (aka data received)
	 * - Set SS pin
	 * - Enable SPI
	 */
	// PA4 (SS)
	GPIOA->MODER |= GPIO_MODER_MODER4_1;
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR4;
	//GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4);
	//GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	GPIOA->AFR[0] |= GPIO_AF6 << GPIO_AFRL_AFRL4_Pos;

	// PC10 (SCL)
	GPIOC->MODER |= GPIO_MODER_MODER10_1;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10;
	//GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_10);
	//GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR10);
	GPIOC->AFR[1] |= GPIO_AF6 << GPIO_AFRH_AFRH2_Pos;

	// PC11 (MISO)
	GPIOC->MODER |= GPIO_MODER_MODER11_1;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11;
	//GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_11);
	//GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR11);
	GPIOC->AFR[1] |= GPIO_AF6 << GPIO_AFRH_AFRH3_Pos;

	// PC12 (MOSI)
	GPIOC->MODER |= GPIO_MODER_MODER12_1;
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12;
	//GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_12);
	//GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR12);
	GPIOC->AFR[1] |= GPIO_AF6 << GPIO_AFRH_AFRH4_Pos;

	// SPI configuration
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN; // Enable SPI on APB bus
	//SPI3->CR1 &= ~SPI_CR1_CPOL; // Reset CPOL
	//SPI3->CR1 &= ~SPI_CR1_CPHA; // Reset CPHA
	//SPI3->CR1 &= ~SPI_CR1_BIDIMODE; // Set as uni-directional (2 line)
	//SPI3->CR1 &= ~SPI_CR1_RXONLY; // Set as full-duplex (4 line)
	//SPI3->CR1 &= ~SPI_CR1_LSBFIRST; // Reset LSB eg. set as MSB
	//SPI3->CR1 &= ~SPI_CR1_SSM; // Set SSM to hardware (micro manages SS itself)
	//SPI3->CR1 |= SPI_CR1_CRCEN; // Enable CRC
	SPI3->CR1 |= 7 << SPI_CR1_BR_Pos; // Set baud rate to Fcpu/256
	SPI3->CR1 |= SPI_CR1_MSTR; // Set as master
	SPI3->CR2 |= 7 << SPI_CR2_DS_Pos; // Set to mode 7, 8-bit
	SPI3->CR2 |= SPI_CR2_FRXTH; // Generate RXNE event when fifo is at 8bit
	SPI3->CR2 |= SPI_CR2_SSOE; // Enable SS output. Allows the micro (as master) to drive the SS pin.

	int i;
	for (;;) {
		SPI_Enable();
		SPI_Send(0xfe);
		SPI_Disable();

		Blink_LED();
		for (i = 0; i < 1000000 / 2; i++) {
			asm("nop");
		}
	}

	return 0;
}

/// Send some data over SPI
/// \param data uint16. The data to send. Max 16 bit but actual transfer size depends on SPI_CR2_DS setting.
///
void SPI_Send(uint16_t data)
{
	/* Wait until previous data has been cleared */
	while (!(SPI3->SR & SPI_SR_TXE));

	/* Send a message (for them to remember) */
	SPI3->DR = data;
}

/// Enable the SPI interface
void SPI_Enable()
{
	SPI3->CR1 |= SPI_CR1_SPE;
}

/// Disable the SPI interface (cleanly)
/// \return uint16. The last data received before closing.
///
uint16_t SPI_Disable()
{
	/* Receive last incoming data. */
	while (!(SPI3->SR & SPI_SR_RXNE));
	uint16_t data = (uint16_t) SPI3->DR;

	/* Wait until outgoing data is sent. */
	while (!(SPI3->SR & SPI_SR_TXE));

	/* Wait until not busy. */
	while (SPI3->SR & SPI_SR_BSY);

	/* Disable the SPI */
	SPI3->CR1 &= ~SPI_CR1_SPE;
	return data;
}

/// \brief Blink the user led
void Blink_LED()
{
	/* Turn on the led */
	GPIOB->BSRR |= (1 << L1_PIN);
	int i;
	for (i = 0; i < 100 * 1000; i++)
			asm("nop");

	GPIOB->BSRR |= (1 << (L1_PIN + 16));
}

/*---------------------------------------------------------------------------*/

/// \brief TIM2 interrupt handler
///
/// Interrupt handler for TIM2, does some led blinking n stuff.
///
void TIM2_IRQHandler()
{
	if ((GPIOB->ODR & (1 << L1_PIN)) == 0) {
		/* Turn on the led */
		GPIOB->BSRR |= (1 << L1_PIN);
	} else {
		/* Turn off the led (reset bit). the BSRR register has reset bits after the set bits, hence why + 16*/
		GPIOB->BSRR |= 1 << (L1_PIN + 16);
		boot_blinks++;
	}

	/* Disable blinking to signal a successful boot. */
	if (boot_blinks == MAX_BOOT_BLINK_COUNT) {
		TIM2->CR1 &= ~(TIM_CR1_CEN);
	}

	/* Clear the interrupt flag */
	TIM2->SR &= ~(TIM_SR_UIF);
}
