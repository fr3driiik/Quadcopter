#ifndef DSHOT_H
#define DSHOT_H
/* 
Example configuration:
motor 1 -> teensy pin 4 -> mxrt1060 pad EMC_06 (H5)
controlled by register IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 at reg 401F_802C (32bit r/w, reset value 0000_0005h) (s435)
and IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_06 at reg 401F_821C (32bit r/w, reset value 0000_10B0h) (s569)
this is connected to FLEXPWM2, FLEXIO1, XBAR_INOUT08 and GPIO4.
We are interested in FLEXPWM2 (eFlexPWM) as output of teensy pin 4, so we need set mux mode to 1.
*/

#include "DMAChannel.h"
#include "Config.h"

#define F_TMR F_BUS_ACTUAL    // 150 000 000hz
#define DSHOT_DSHOT_LENGTH 16
#define DSHOT_DMA_MARGIN 2  // Number of additional bit duration to wait until checking if DMA is over
#define DSHOT_DMA_LENGTH 18
#define DSHOT_OUTPUTS 4

#define DSHOT_BT_DURATION 1000000000 / DSHOT_BITRATE  // Duration of 1 DSHOT bit in ns
#define DSHOT_LP_DURATION DSHOT_BT_DURATION * 0.75    // Duration of a DSHOT long pulse in ns
#define DSHOT_SP_DURATION DSHOT_BT_DURATION * 0.375   // Duration of a DSHOT short pulse in ns

#define DSHOT_MAX_VALUE 2047  // Maximum DSHOT value
#define DSHOT_CMD_MAX   47

#define DSHOT_ERROR_DMA -1        // DMA error
#define DSHOT_ERROR_TIMEOUT -2    // Timeout : DMA duration is abnormally great
#define DSHOT_ERROR_RANGE -3      // Value out of range
#define DSHOT_ERROR_INTERNAL -4   // Internal error
#define DSHOT_ERROR_NOT_READY -5  // send request issued before previous send has completed

const uint16_t DSHOT_short_pulse = uint64_t(F_TMR) * DSHOT_SP_DURATION / 1000000000;  // DSHOT short pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_long_pulse = uint64_t(F_TMR) * DSHOT_LP_DURATION / 1000000000;   // DSHOT long pulse duration (nb of F_BUS periods)
const uint16_t DSHOT_bit_length = uint64_t(F_TMR) * DSHOT_BT_DURATION / 1000000000;   // DSHOT bit duration (nb of F_BUS periods)

// DMA eFlexPWM modules
volatile IMXRT_FLEXPWM_t *DSHOT_pwm_module[DSHOT_OUTPUTS] = {
  &IMXRT_FLEXPWM2,
  &IMXRT_FLEXPWM1,
  &IMXRT_FLEXPWM2,
  &IMXRT_FLEXPWM2
};

// DMA eFlexPWM submodules
volatile uint8_t DSHOT_pwm_submodule[DSHOT_OUTPUTS] = { 0, 3, 1, 2 };

// DMA eFlexPWM submodule PWM channel selector: A=0, B=1, X=2
volatile uint8_t DSHOT_pwm_channel[DSHOT_OUTPUTS] = { 0, 0, 0, 1 };

// Output pins
volatile uint8_t DSHOT_pin[DSHOT_OUTPUTS] = { ESC_FL_PIN, ESC_FR_PIN, ESC_RL_PIN, ESC_RR_PIN };

// Output pin mux mode
volatile uint8_t DSHOT_pinmux[DSHOT_OUTPUTS] = { 1, 6, 1, 2 };

// DMA source
volatile uint8_t DSHOT_dmamux[DSHOT_OUTPUTS] = {
  DMAMUX_SOURCE_FLEXPWM2_WRITE0,
  DMAMUX_SOURCE_FLEXPWM1_WRITE3,
  DMAMUX_SOURCE_FLEXPWM2_WRITE1,
  DMAMUX_SOURCE_FLEXPWM2_WRITE2
};

volatile uint16_t DSHOT_dma_data[DSHOT_OUTPUTS][DSHOT_DMA_LENGTH];
DMAChannel dma[DSHOT_OUTPUTS];

/* 
 * DMA termination interrupt service routine (ISR) for each DMA channel
 */
#define DSHOT_DMA_interrupt_routine(DSHOT_OUTPUT) \
  void DSHOT_DMA_interrupt_routine_##DSHOT_OUTPUT(void) { \
    dma[DSHOT_OUTPUT].clearInterrupt(); \
    (*DSHOT_pwm_module[DSHOT_OUTPUT]).MCTRL &= FLEXPWM_MCTRL_RUN(~(1 << DSHOT_pwm_submodule[DSHOT_OUTPUT])); \
  }

DSHOT_DMA_interrupt_routine(0);
DSHOT_DMA_interrupt_routine(1);
DSHOT_DMA_interrupt_routine(2);
DSHOT_DMA_interrupt_routine(3);

void (*DSHOT_DMA_ISR[DSHOT_OUTPUTS])() = {
  DSHOT_DMA_interrupt_routine_0,
  DSHOT_DMA_interrupt_routine_1,
  DSHOT_DMA_interrupt_routine_2,
  DSHOT_DMA_interrupt_routine_3,
};

void DSHOT_init() {
  int i, j;

  for (i = 0; i < DSHOT_OUTPUTS; i++) {
    // Initialize DMA data
    for (j = 0; j < DSHOT_DMA_LENGTH; j++) {
      DSHOT_dma_data[i][j] = 0;
    }

    // Configure pins on the board as DSHOT outputs
    // These pins are configured as eFlexPWM (FLEXPWMn) PWM outputs
    *(portConfigRegister(DSHOT_pin[i])) = DSHOT_pinmux[i];

    // Configure eFlexPWM modules and submodules for PWM generation
    // --- submodule specific registers ---
    // INIT: initial counter value
    // VAL0: PWM_X compare value
    // VAL1: counter max value
    // VAL2: must be 0 for edge-aligned PWM
    // VAL3: PWM_A compare value
    // VAL4: must be 0 for edge-aligned PWM
    // VAL5: PWM_B compare value
    // OCTRL: invert polarity of PWMq FLEXPWM_SMOCTRL_POLq
    // DMAEN: FLEXPWM_SMDMAEN_VALDE to enable DMA
    // --- module specific registers ---
    // OUTEN: output enable for submodule n and PWM q FLEXPWM_OUTEN_PWMq_EN( 1 << n )
    (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].INIT = 0;
    (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL0 = 0;
    (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL1 = DSHOT_bit_length;
    (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL2 = 0;
    (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL3 = 0;
    (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL4 = 0;
    (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL5 = 0;
    if (DSHOT_pwm_channel[i] == 2) {
      (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].OCTRL = FLEXPWM_SMOCTRL_POLX;
      (*DSHOT_pwm_module[i]).OUTEN |= FLEXPWM_OUTEN_PWMX_EN(1 << DSHOT_pwm_submodule[i]);
    } else if (DSHOT_pwm_channel[i] == 1) {
      (*DSHOT_pwm_module[i]).OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << DSHOT_pwm_submodule[i]);
    } else {
      (*DSHOT_pwm_module[i]).OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << DSHOT_pwm_submodule[i]);
    }
    (*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].DMAEN = FLEXPWM_SMDMAEN_VALDE;

    // Each DMA channel is linked to a unique eFlexPWM submodule
    // DMA channels are triggered by independant hardware events
    dma[i].sourceBuffer(DSHOT_dma_data[i], DSHOT_DMA_LENGTH * sizeof(uint16_t));
    if (DSHOT_pwm_channel[i] == 2) {
      dma[i].destination((uint16_t &)(*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL0);
    } else if (DSHOT_pwm_channel[i] == 1) {
      dma[i].destination((uint16_t &)(*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL5);
    } else {
      dma[i].destination((uint16_t &)(*DSHOT_pwm_module[i]).SM[DSHOT_pwm_submodule[i]].VAL3);
    }
    dma[i].triggerAtHardwareEvent(DSHOT_dmamux[i]);  // when pwm timer is done, we send next dshot bit pwm value
    dma[i].interruptAtCompletion();
    dma[i].attachInterrupt(DSHOT_DMA_ISR[i]);
    dma[i].enable();
  }
}

int DSHOT_send(uint16_t *cmd, uint8_t *tlm) {
  int i, j;
  uint16_t data;
  bool error = false;

  for (i = 0; i < DSHOT_OUTPUTS; i++) {
    // Check if there is a DMA error from previous loop
    if (dma[i].error()) {
      error = true;
    }

    if (cmd[i] > DSHOT_MAX_VALUE) {
      return DSHOT_ERROR_RANGE;
    }

    data = (cmd[i] << 5) | (tlm[i] << 4);
    data |= ((data >> 4) ^ (data >> 8) ^ (data >> 12)) & 0x0f;

    // Generate DSHOT timings corresponding to the packet
    for (j = 0; j < DSHOT_DSHOT_LENGTH; j++) {
      if (data & (1 << (DSHOT_DSHOT_LENGTH - 1 - j))) {
        DSHOT_dma_data[i][j] = DSHOT_long_pulse;
      } else {
        DSHOT_dma_data[i][j] = DSHOT_short_pulse;
      }
    }

    // Clear error flag on all DMA channels
    dma[i].clearError();

    // Start DMA by activating the clocks
    // Clocks are disabled again by the DMA ISRs
    (*DSHOT_pwm_module[i]).MCTRL |= FLEXPWM_MCTRL_RUN(1 << DSHOT_pwm_submodule[i]);
  }

  if (error) {
    return DSHOT_ERROR_DMA;
  }
  // Wait the theoretical time needed by DMA + some margin
  // TODO: hmm this is stupid with DMA, this is part of what we are aiming to gain, extra CPU time. Instead make sure that enough time has passed when calling this func or return NOT_READY error
  delayMicroseconds((unsigned int)((DSHOT_BT_DURATION * (DSHOT_DMA_LENGTH + DSHOT_DMA_MARGIN)) / 1000));


  return 0;
}

#endif