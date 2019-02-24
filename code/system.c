#include <stm32f3xx.h>

void SystemInit(void)
{
	/* Reset the RCC clock configuration to the default reset state ------------*/
	/* Set HSION bit */
	RCC->CR |= 0x00000001U;

	/* Reset CFGR register */
	RCC->CFGR &= 0xF87FC00CU;

	/* Reset HSEON, CSSON and PLLON bits */
	RCC->CR &= 0xFEF6FFFFU;

	/* Reset HSEBYP bit */
	RCC->CR &= 0xFFFBFFFFU;

	/* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE bits */
	RCC->CFGR &= 0xFF80FFFFU;

	/* Reset PREDIV1[3:0] bits */
	RCC->CFGR2 &= 0xFFFFFFF0U;

	/* Reset USARTSW[1:0], I2CSW and TIMs bits */
	RCC->CFGR3 &= 0xFF00FCCCU;

	/* Disable all interrupts */
	RCC->CIR = 0x00000000U;

	/* Initialize the MCU */
	/* Wait until HSI (Internal oscillator) is ready */
	while ((RCC->CR & RCC_CR_HSIRDY) == 0);

	/* Disable PLL */
	RCC->CR &= ~(RCC_CR_PLLON);
	/* Wait until PLL ready (disabled) */
	while ((RCC->CR & RCC_CR_PLLRDY) != 0);

	/*
	 * Configure PLL
	 *
	 * Does some advanced clock manipulation "things" so we can generate a clock higher than
	 * the oscillator provides. In this case 8MHz -> 52MHz.
	 * For the STM32F302r8 the internal oscillator gets divided by 2 before it enters the PLL.
	 *
	 * HSI as clock input
	 * Note: For some reason 16x doesn't work, even though it only adds up to 64MHz (out of 72).
	 * Note 2: "works" at 14x as well but sometimes wont boot. Not sure what's the problem.
	 * PPLMUL = 13x		Multiply HSI (8/2 = 4MHz) by 13 to get 52MHz
	 * PPRE1 = HCLK/2	Divide APB1 clock to get it to 26MHz (Max 72/2 = 36)
	 */
	RCC->CFGR |= RCC_CFGR_PLLMUL2 | RCC_CFGR_PPRE1_DIV2;

	// Additional possible configurations
	//	/* Set HCLK (AHB1) prescaler (DIV1) */
	//	RCC->CFGR &= ~(RCC_CFGR_HPRE);
	//
	//	/* Set APB1 Low speed prescaler (APB1) DIV2 */
	//	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	//
	//	/* SET APB2 High speed srescaler (APB2) DIV1 */
	//	RCC->CFGR &= ~(RCC_CFGR_PPRE2);

	/* PLL On */
	RCC->CR |= RCC_CR_PLLON;
	/* Wait until PLL is ready */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	/*
	 * FLASH configuration block
	 * enable instruction cache
	 * enable prefetch
	 * set latency to 2WS (3 CPU cycles)
	 */
	// FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_2WS;

	/* Check flash latency */
	//if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2WS) {
	//	SystemInitError(SYSTEM_INIT_ERROR_FLASH);
	//}

	/* Set clock source to PLL */
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	/* Check clock source */
	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL);

}
