#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "cpu.h"
#include "cmu.h"
#include "em_chip.h"
#include "os.h"
#include "bsp_os.h"
#include "os_trace.h"
#include "em_emu.h"

int main(void)
{
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_DEFAULT;
  CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;

  /* Chip errata */
  CHIP_Init();

  /* Init DCDC regulator and HFXO with kit specific parameters */
  /* Init DCDC regulator and HFXO with kit specific parameters */
  /* Initialize DCDC. Always start in low-noise mode. */
  EMU_EM23Init_TypeDef em23Init = EMU_EM23INIT_DEFAULT;
  EMU_DCDCInit(&dcdcInit);
  em23Init.vScaleEM23Voltage = emuVScaleEM23_LowPower;
  EMU_EM23Init(&em23Init);
  CMU_HFXOInit(&hfxoInit);

  /* Switch HFCLK to HFRCO and disable HFRCO */
  CMU_OscillatorEnable(cmuOsc_HFRCO, true, true);
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFRCO);
  CMU_OscillatorEnable(cmuOsc_HFXO, false, false);

  cmu_open();

  BSP_SystemInit();                                           /* Initialize System.                                   */
  RTOS_ERR  err;

  CPU_Init();
  OS_TRACE_INIT();
  OSInit(&err);                                               /* Initialize the Kernel.                               */
  /*   Check error code.                                  */
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  app_init();


  OSStart(&err);                                              /* Start the kernel.                                    */
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
