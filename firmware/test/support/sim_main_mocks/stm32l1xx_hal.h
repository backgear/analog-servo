/*
 * Minimal host-side mock of the STM32 HAL -- ONLY for
 * simmy_commutation_learn_integration_main.c's build of the REAL main.c
 * (see ../simmy_commutation_learn_integration_main.c). A separate, DEDICATED
 * mock directory from both test/mocks/ (used by `pio test -e native`'s
 * Unity suites, which never build main.c at all) and
 * test/support/sim_foc_mocks/ (the OTHER host-integration driver's mock,
 * which only ever needed to satisfy Output.c's DAC calls, not a GPIO/DWT
 * surface this big) -- this is the first thing to ever compile the real
 * main.c natively, and main.c's translation unit references a much wider
 * HAL surface (GPIO ports/pins, the DWT cycle counter) than Output.c alone
 * ever did. Kept separate rather than folded into either existing mock set
 * so neither of those gets a wider surface than it actually needs.
 *
 * Nothing here models real GPIO/DAC/DWT electrical behavior -- the actual
 * simulation-driving logic lives in
 * simmy_commutation_learn_integration_main.c's HAL_GPIO_ReadPin()/
 * HAL_DAC_SetValue()/PhaseSignWrite() DEFINITIONS (this header only
 * declares the shapes main.c/Output.c expect to find).
 */
#ifndef MOCK_STM32L1XX_HAL_H
#define MOCK_STM32L1XX_HAL_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

/* CMSIS intrinsics -- main.c's own real Error_Handler()/AppResetPosition()
 * call these directly; host-side no-ops (nothing to actually
 * disable/save/restore without a real NVIC). */
#define __disable_irq() ((void)0)
#define __get_PRIMASK() 0U
#define __set_PRIMASK(x) ((void)(x))

/* ---- CMSIS Cortex-M3 core / RCC / PWR / FLASH -- SystemClock_Config()'s
 * and real main()'s own HAL init sequence reference these directly, so
 * the WHOLE FILE can compile (gcc type-checks every function regardless
 * of whether `-Wl,--gc-sections` will later drop it from the link --
 * that flag only prunes the link step, not compilation). None of this is
 * ever actually exercised: SystemClock_Config()/the renamed real main()
 * (see this driver's own `-Dmain=` build flag) are never called, only
 * compiled, by this test -- these are dummy shapes/values, not a claim
 * about real register behavior. */
typedef struct { volatile uint32_t DEMCR; } CoreDebugType;
extern CoreDebugType gMockCoreDebug;
#define CoreDebug (&gMockCoreDebug)
#define CoreDebug_DEMCR_TRCENA_Msk 0x01000000U
#define DWT_CTRL_CYCCNTENA_Msk     0x00000001U

HAL_StatusTypeDef HAL_Init(void);

#define RESET 0U
#define PWR_FLAG_VOS 0U
#define PWR_REGULATOR_VOLTAGE_SCALE1 0U
#define __HAL_RCC_PWR_CLK_ENABLE() ((void)0)
#define __HAL_PWR_VOLTAGESCALING_CONFIG(x) ((void)(x))
#define __HAL_PWR_GET_FLAG(x) ((void)(x), 1U)   /* != RESET immediately -- SystemClock_Config()'s
                                                      wait loop is dead code here, never runs */

typedef struct {
  uint32_t PLLState;
  uint32_t PLLSource;
  uint32_t PLLMUL;
  uint32_t PLLDIV;
} RCC_PLLInitTypeDef;
typedef struct {
  uint32_t OscillatorType;
  uint32_t HSEState;
  RCC_PLLInitTypeDef PLL;
} RCC_OscInitTypeDef;
typedef struct {
  uint32_t ClockType;
  uint32_t SYSCLKSource;
  uint32_t AHBCLKDivider;
  uint32_t APB1CLKDivider;
  uint32_t APB2CLKDivider;
} RCC_ClkInitTypeDef;
#define RCC_OSCILLATORTYPE_HSE 0U
#define RCC_HSE_BYPASS 0U
#define RCC_PLL_ON 0U
#define RCC_PLLSOURCE_HSE 0U
#define RCC_PLL_MUL12 0U
#define RCC_PLL_DIV3 0U
#define RCC_CLOCKTYPE_HCLK 0U
#define RCC_CLOCKTYPE_SYSCLK 0U
#define RCC_CLOCKTYPE_PCLK1 0U
#define RCC_CLOCKTYPE_PCLK2 0U
#define RCC_SYSCLKSOURCE_PLLCLK 0U
#define RCC_SYSCLK_DIV1 0U
#define RCC_HCLK_DIV1 0U
#define FLASH_LATENCY_1 0U
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *osc);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *clk, uint32_t flashLatency);

/* ---- GPIO: just enough of ST's HAL_GPIO shapes for main.c/gpio.h's calls.
 * GPIOx are arbitrary distinct non-null sentinels -- main.c never
 * dereferences them, only compares which port/pin pair it was given
 * (see HAL_GPIO_ReadPin()'s real definition in the test driver). */
typedef struct { int unused; } GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef *)1)
#define GPIOB ((GPIO_TypeDef *)2)
#define GPIOC ((GPIO_TypeDef *)3)

#define GPIO_PIN_0  (1U << 0)
#define GPIO_PIN_1  (1U << 1)
#define GPIO_PIN_2  (1U << 2)
#define GPIO_PIN_3  (1U << 3)
#define GPIO_PIN_4  (1U << 4)
#define GPIO_PIN_5  (1U << 5)
#define GPIO_PIN_6  (1U << 6)
#define GPIO_PIN_7  (1U << 7)
#define GPIO_PIN_8  (1U << 8)
#define GPIO_PIN_9  (1U << 9)
#define GPIO_PIN_10 (1U << 10)
#define GPIO_PIN_11 (1U << 11)
#define GPIO_PIN_12 (1U << 12)
#define GPIO_PIN_13 (1U << 13)
#define GPIO_PIN_14 (1U << 14)
#define GPIO_PIN_15 (1U << 15)

typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;

GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void          HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

/* ---- DWT cycle counter -- AppControlTick() reads DWT->CYCCNT directly
 * (dtCycles = nowCycles - sLastCycles). Value doesn't need to be "real"
 * cycles, just monotonically advancing by a consistent per-tick amount --
 * see the test driver's own per-tick loop, which advances it externally
 * before each AppControlTick() call. */
typedef struct { volatile uint32_t CTRL; volatile uint32_t CYCCNT; } MockDwtType;
extern MockDwtType gMockDwt;
#define DWT (&gMockDwt)

/* ---- DAC: just enough of ST's HAL_DAC shapes for Output.c's calls, same
 * shapes as test/support/sim_foc_mocks/stm32l1xx_hal.h's own (duplicated
 * rather than shared -- see this file's own top comment for why these two
 * mock sets stay separate). */
typedef struct { int unused; } DAC_TypeDef;
#define DAC ((DAC_TypeDef *)0)

typedef struct {
  DAC_TypeDef *Instance;
} DAC_HandleTypeDef;

typedef struct {
  uint32_t DAC_Trigger;
  uint32_t DAC_OutputBuffer;
} DAC_ChannelConfTypeDef;

#define DAC_CHANNEL_1          0U
#define DAC_CHANNEL_2          1U
#define DAC_ALIGN_12B_R        0U
#define DAC_TRIGGER_NONE       0U
#define DAC_OUTPUTBUFFER_ENABLE 0U
#define __HAL_RCC_DAC_CLK_ENABLE() ((void)0)

HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac,
                                        DAC_ChannelConfTypeDef *config, uint32_t channel);
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t channel);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t channel,
                                   uint32_t alignment, uint32_t value);

#endif /* MOCK_STM32L1XX_HAL_H */
