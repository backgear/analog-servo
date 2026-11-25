/*
 * Minimal host-side mock of the STM32 HAL -- ONLY for
 * simmy_foc_integration_main.c's build of the REAL Output.c (see
 * ../simmy_foc_integration_main.c and docs/two-phase-drive.md). A separate
 * file from test/mocks/stm32l1xx_hal.h (used by `pio test -e native`'s
 * Unity suites) on purpose: that one is deliberately kept minimal because
 * none of today's native Unity tests build Output.c; this driver is the
 * first thing that does, and it needs the DAC peripheral shapes
 * Output.c's OutputInit()/OutputWriteDac()/OutputWriteDacTwoPhase() call
 * into. Touching the shared mock to add these would risk the existing
 * Unity suites for no benefit to them -- this file is additive and scoped
 * to this one driver's own include path instead.
 *
 * Nothing here models real DAC timing/electrical behavior -- the actual
 * capture-and-reconstruct logic lives in simmy_foc_integration_main.c's
 * HAL_DAC_SetValue()/PhaseSignWrite() DEFINITIONS (this header only
 * declares the shapes Output.c expects to find).
 */
#ifndef MOCK_STM32L1XX_HAL_H
#define MOCK_STM32L1XX_HAL_H

#include <stdint.h>
#include <stddef.h>

typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;

/* ---- DAC: just enough of ST's HAL_DAC shapes for Output.c's calls ------- */
typedef struct { int unused; } DAC_TypeDef;
#define DAC ((DAC_TypeDef *)0)   /* real register base address on the MCU;
                                     an arbitrary non-dereferenced value
                                     host-side -- OutputInit() only ever
                                     assigns it to sHdac.Instance, never
                                     reads through it in this mock. */

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

/* Real register-enable macro on the MCU; a no-op host-side -- nothing to
 * clock-gate when there's no peripheral. */
#define __HAL_RCC_DAC_CLK_ENABLE() ((void)0)

HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac,
                                        DAC_ChannelConfTypeDef *config, uint32_t channel);
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t channel);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t channel,
                                   uint32_t alignment, uint32_t value);

#endif /* MOCK_STM32L1XX_HAL_H */
