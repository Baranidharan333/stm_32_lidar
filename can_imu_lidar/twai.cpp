/**
 * twai.cpp  –  ESP-IDF style TWAI driver on STM32 bxCAN (CAN1)
 *
 * TX: software queue → 3 hardware mailboxes (sent in request order).
 * RX: FIFO0 → software queue.
 * Errors: SCE interrupt + polling of CAN1->ESR produce the TWAI alerts.
 */

#include "twai.h"
#include "pinmap.h"

#if !defined(CAN1)
#error "twai driver needs an STM32 with a bxCAN (CAN1) peripheral"
#endif

#define TWAI_TX_QUEUE_MAX 32
#define TWAI_RX_QUEUE_MAX 128
#define TWAI_IRQ_PRIORITY 5

#define ALL_TME   (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)
#define ALL_ABRQ  (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2)
#define ESR_FLAGS (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF)

// ─── Driver state ────────────────────────────────────────────────────────────
static bool installed = false;
static bool autoRecovery = false;
static volatile twai_state_t state = TWAI_STATE_STOPPED;
static twai_mode_t mode = TWAI_MODE_NORMAL;

static volatile uint32_t alertsEnabled = 0;
static volatile uint32_t alertsPending = 0;
static volatile uint32_t lastEsrFlags = 0;

static twai_message_t txQueue[TWAI_TX_QUEUE_MAX];
static volatile uint16_t txHead = 0, txCount = 0;
static uint16_t txLen = 0;

static twai_message_t rxQueue[TWAI_RX_QUEUE_MAX];
static volatile uint16_t rxHead = 0, rxCount = 0;
static uint16_t rxLen = 0;

static volatile uint32_t txSuccessCount = 0;
static volatile uint32_t txFailedCount = 0;
static volatile uint32_t rxMissedCount = 0;
static volatile uint32_t rxOverrunCount = 0;
static volatile uint32_t arbLostCount = 0;
static volatile uint32_t busErrorCount = 0;

// ─── Helpers ─────────────────────────────────────────────────────────────────

static inline void raiseAlert(uint32_t alert) {
  alertsPending |= alert & alertsEnabled;
}

static bool waitMcrAck(bool initMode, uint32_t timeoutMs) {
  uint32_t start = millis();
  while (((CAN1->MSR & CAN_MSR_INAK) != 0) != initMode) {
    if (millis() - start >= timeoutMs) {
      return false;
    }
  }
  return true;
}

// Pick BRP / TSEG1 / TSEG2 for the bitrate from the APB1 clock.
static bool computeBtr(uint32_t pclk, const twai_timing_config_t *t, uint32_t *btr) {
  if (t->bitrate == 0) {
    return false;
  }
  uint32_t sp = t->sample_point_permille ? t->sample_point_permille : 875;

  for (uint32_t tq = 25; tq >= 8; tq--) {
    if (pclk % (t->bitrate * tq) != 0) {
      continue;
    }
    uint32_t brp = pclk / (t->bitrate * tq);
    if (brp < 1 || brp > 1024) {
      continue;
    }

    // Sample point = (1 + TSEG1) / tq
    uint32_t tseg1 = (tq * sp + 500) / 1000 - 1;
    if (tseg1 > 16) tseg1 = 16;
    uint32_t tseg2 = tq - 1 - tseg1;
    if (tseg2 < 1 || tseg2 > 8) {
      continue;
    }

    uint32_t sjw = t->sjw ? t->sjw : 1;
    if (sjw > tseg2) sjw = tseg2;
    if (sjw > 4) sjw = 4;

    *btr = ((sjw - 1) << CAN_BTR_SJW_Pos) |
           ((tseg2 - 1) << CAN_BTR_TS2_Pos) |
           ((tseg1 - 1) << CAN_BTR_TS1_Pos) |
           (brp - 1);
    return true;
  }
  return false;
}

static void writeMailbox(uint32_t index, const twai_message_t *msg) {
  CAN_TxMailBox_TypeDef *mb = &CAN1->sTxMailBox[index];

  uint32_t tir = msg->extd
                 ? ((msg->identifier & TWAI_EXTD_ID_MASK) << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE
                 : ((msg->identifier & TWAI_STD_ID_MASK) << CAN_TI0R_STID_Pos);
  if (msg->rtr) {
    tir |= CAN_TI0R_RTR;
  }

  const uint8_t *d = msg->data;
  mb->TDTR = msg->data_length_code & 0x0F;
  mb->TDLR = d[0] | (d[1] << 8) | (d[2] << 16) | ((uint32_t)d[3] << 24);
  mb->TDHR = d[4] | (d[5] << 8) | (d[6] << 16) | ((uint32_t)d[7] << 24);
  mb->TIR  = tir | CAN_TI0R_TXRQ;
}

// Move queued frames into free mailboxes. Call with the TX IRQ masked.
static void fillMailboxes(void) {
  while (txCount > 0 && (CAN1->TSR & ALL_TME)) {
    uint32_t index = (CAN1->TSR & CAN_TSR_CODE) >> CAN_TSR_CODE_Pos;
    writeMailbox(index, &txQueue[txHead]);
    txHead = (txHead + 1) % txLen;
    txCount--;
  }
}

static void abortTransmissions(void) {
  txHead = 0;
  txCount = 0;
  CAN1->TSR = ALL_ABRQ;
}

// Turn changes of EWGF / EPVF / BOFF into alerts and state changes.
// Called from the SCE interrupt and (with interrupts off) from the API.
static void updateErrorState(uint32_t esr) {
  uint32_t now = esr & ESR_FLAGS;
  uint32_t changed = now ^ lastEsrFlags;
  lastEsrFlags = now;

  if (changed & CAN_ESR_EWGF) {
    raiseAlert((now & CAN_ESR_EWGF) ? TWAI_ALERT_ABOVE_ERR_WARN : TWAI_ALERT_BELOW_ERR_WARN);
  }

  if (changed & CAN_ESR_EPVF) {
    if (now & CAN_ESR_EPVF) {
      raiseAlert(TWAI_ALERT_ERR_PASS);
    } else if (!(now & CAN_ESR_BOFF)) {
      raiseAlert(TWAI_ALERT_ERR_ACTIVE);
    }
  }

  if ((changed & CAN_ESR_BOFF) && (now & CAN_ESR_BOFF)) {
    // Entered bus-off: drop everything waiting to be sent (like ESP-IDF).
    state = TWAI_STATE_BUS_OFF;
    abortTransmissions();
    raiseAlert(TWAI_ALERT_BUS_OFF);
  }

  if ((changed & CAN_ESR_BOFF) && !(now & CAN_ESR_BOFF)) {
    // 128 x 11 recessive bits seen: TEC / REC are back to 0.
    if (state == TWAI_STATE_RECOVERING) {
      // Manual recovery: go to STOPPED, the user calls twai_start().
      CAN1->MCR |= CAN_MCR_INRQ;
      state = TWAI_STATE_STOPPED;
      raiseAlert(TWAI_ALERT_BUS_RECOVERED);
    } else if (state == TWAI_STATE_BUS_OFF) {
      // Hardware auto recovery (ABOM): straight back to RUNNING.
      state = TWAI_STATE_RUNNING;
      raiseAlert(TWAI_ALERT_BUS_RECOVERED);
    }
  }
}

// Leaving bus-off does not raise an interrupt, so the API polls ESR too.
static void pollErrorState(void) {
  if (!installed) {
    return;
  }
  noInterrupts();
  updateErrorState(CAN1->ESR);
  interrupts();
}

static void enableIrqs(bool enable) {
  const IRQn_Type irqs[] = { CAN1_TX_IRQn, CAN1_RX0_IRQn, CAN1_SCE_IRQn };
  for (IRQn_Type irq : irqs) {
    if (enable) {
      NVIC_SetPriority(irq, TWAI_IRQ_PRIORITY);
      NVIC_ClearPendingIRQ(irq);
      NVIC_EnableIRQ(irq);
    } else {
      NVIC_DisableIRQ(irq);
    }
  }
}

// ─── Interrupt handlers ──────────────────────────────────────────────────────

extern "C" void CAN1_TX_IRQHandler(void) {
  uint32_t tsr = CAN1->TSR;

  for (uint32_t i = 0; i < 3; i++) {
    uint32_t shift = 8 * i;
    if (!(tsr & (CAN_TSR_RQCP0 << shift))) {
      continue;
    }

    if (tsr & (CAN_TSR_TXOK0 << shift)) {
      txSuccessCount++;
      raiseAlert(TWAI_ALERT_TX_SUCCESS);
    } else {
      if (tsr & (CAN_TSR_ALST0 << shift)) {
        arbLostCount++;
        raiseAlert(TWAI_ALERT_ARB_LOST);
      }
      txFailedCount++;
      raiseAlert(TWAI_ALERT_TX_FAILED);
    }

    // Writing RQCP clears RQCP, TXOK, ALST and TERR of that mailbox.
    CAN1->TSR = CAN_TSR_RQCP0 << shift;
  }

  fillMailboxes();

  if (txCount == 0 && (CAN1->TSR & ALL_TME) == ALL_TME) {
    raiseAlert(TWAI_ALERT_TX_IDLE);
  }
}

extern "C" void CAN1_RX0_IRQHandler(void) {
  while (CAN1->RF0R & CAN_RF0R_FMP0) {
    CAN_FIFOMailBox_TypeDef *mb = &CAN1->sFIFOMailBox[0];
    uint32_t rir  = mb->RIR;
    uint32_t rdtr = mb->RDTR;
    uint32_t rdlr = mb->RDLR;
    uint32_t rdhr = mb->RDHR;
    CAN1->RF0R = CAN_RF0R_RFOM0;  // release the FIFO slot

    if (rxCount >= rxLen) {
      rxMissedCount++;
      raiseAlert(TWAI_ALERT_RX_QUEUE_FULL);
      continue;
    }

    twai_message_t *m = &rxQueue[(rxHead + rxCount) % rxLen];
    m->flags = 0;
    if (rir & CAN_RI0R_IDE) {
      m->extd = 1;
      m->identifier = (rir >> CAN_RI0R_EXID_Pos) & TWAI_EXTD_ID_MASK;
    } else {
      m->identifier = (rir >> CAN_RI0R_STID_Pos) & TWAI_STD_ID_MASK;
    }
    m->rtr = (rir & CAN_RI0R_RTR) ? 1 : 0;
    m->data_length_code = rdtr & CAN_RDT0R_DLC;
    m->dlc_non_comp = m->data_length_code > TWAI_FRAME_MAX_DLC;
    for (int b = 0; b < 4; b++) {
      m->data[b]     = rdlr >> (8 * b);
      m->data[b + 4] = rdhr >> (8 * b);
    }
    rxCount++;
    raiseAlert(TWAI_ALERT_RX_DATA);
  }

  if (CAN1->RF0R & CAN_RF0R_FOVR0) {
    rxOverrunCount++;
    raiseAlert(TWAI_ALERT_RX_FIFO_OVERRUN);
    CAN1->RF0R = CAN_RF0R_FOVR0;
  }
}

extern "C" void CAN1_SCE_IRQHandler(void) {
  uint32_t esr = CAN1->ESR;

  if (esr & CAN_ESR_LEC) {
    busErrorCount++;
    raiseAlert(TWAI_ALERT_BUS_ERROR);
    CAN1->ESR = 0;  // clear LEC (the rest of ESR is read-only)
  }

  updateErrorState(esr);
  CAN1->MSR = CAN_MSR_ERRI;
}

// ─── API ─────────────────────────────────────────────────────────────────────

esp_err_t twai_driver_install(const twai_general_config_t *g_config,
                              const twai_timing_config_t *t_config,
                              const twai_filter_config_t *f_config) {
  if (!g_config || !t_config || !f_config) {
    return ESP_ERR_INVALID_ARG;
  }
  if (installed) {
    return ESP_ERR_INVALID_STATE;
  }

  uint32_t btr;
  if (!computeBtr(HAL_RCC_GetPCLK1Freq(), t_config, &btr)) {
    return ESP_ERR_INVALID_ARG;
  }

  // Pins: CAN1 is AF9 on all its pin options of the F2/F4
  PinName txPin = digitalPinToPinName(g_config->tx_io);
  PinName rxPin = digitalPinToPinName(g_config->rx_io);
  if (txPin == NC || rxPin == NC) {
    return ESP_ERR_INVALID_ARG;
  }
  pin_function(txPin, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_NOPULL, GPIO_AF9_CAN1));
  pin_function(rxPin, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF9_CAN1));

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_CAN1_FORCE_RESET();
  __HAL_RCC_CAN1_RELEASE_RESET();

  // Wake up and enter initialization mode
  CAN1->MCR = CAN_MCR_INRQ;
  if (!waitMcrAck(true, 10)) {
    return ESP_FAIL;
  }

  // TXFP: mailboxes go out in request order. ABOM: hardware bus-off recovery.
  mode = g_config->mode;
  autoRecovery = g_config->auto_bus_off_recovery;
  CAN1->MCR = CAN_MCR_INRQ | CAN_MCR_TXFP | (autoRecovery ? CAN_MCR_ABOM : 0);

  switch (mode) {
    case TWAI_MODE_NO_ACK:      btr |= CAN_BTR_LBKM; break;
    case TWAI_MODE_LISTEN_ONLY: btr |= CAN_BTR_SILM; break;
    case TWAI_MODE_LOOPBACK:    btr |= CAN_BTR_LBKM | CAN_BTR_SILM; break;
    default: break;
  }
  CAN1->BTR = btr;

  // Filter bank 0: one 32-bit mask filter into FIFO0.
  // bxCAN mask bit 1 = must match, the TWAI mask bit 1 = don't care.
  CAN1->FMR |= CAN_FMR_FINIT;
  CAN1->FA1R &= ~CAN_FA1R_FACT0;
  CAN1->FS1R |= CAN_FS1R_FSC0;
  CAN1->FM1R &= ~CAN_FM1R_FBM0;
  CAN1->FFA1R &= ~CAN_FFA1R_FFA0;
  CAN1->sFilterRegister[0].FR1 = f_config->acceptance_code & ~f_config->acceptance_mask;
  CAN1->sFilterRegister[0].FR2 = ~f_config->acceptance_mask;
  CAN1->FA1R |= CAN_FA1R_FACT0;
  CAN1->FMR &= ~CAN_FMR_FINIT;

  txLen = constrain(g_config->tx_queue_len, 1, TWAI_TX_QUEUE_MAX);
  rxLen = constrain(g_config->rx_queue_len, 1, TWAI_RX_QUEUE_MAX);
  txHead = txCount = 0;
  rxHead = rxCount = 0;
  txSuccessCount = txFailedCount = rxMissedCount = rxOverrunCount = arbLostCount = busErrorCount = 0;
  alertsEnabled = g_config->alerts_enabled;
  alertsPending = 0;
  lastEsrFlags = 0;
  state = TWAI_STATE_STOPPED;

  CAN1->IER = CAN_IER_TMEIE | CAN_IER_FMPIE0 | CAN_IER_FOVIE0 |
              CAN_IER_ERRIE | CAN_IER_EWGIE | CAN_IER_EPVIE |
              CAN_IER_BOFIE | CAN_IER_LECIE;
  enableIrqs(true);

  installed = true;
  return ESP_OK;
}

esp_err_t twai_driver_uninstall(void) {
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }
  if (state == TWAI_STATE_RUNNING || state == TWAI_STATE_RECOVERING) {
    return ESP_ERR_INVALID_STATE;
  }

  enableIrqs(false);
  CAN1->IER = 0;
  __HAL_RCC_CAN1_FORCE_RESET();
  __HAL_RCC_CAN1_RELEASE_RESET();
  __HAL_RCC_CAN1_CLK_DISABLE();

  installed = false;
  return ESP_OK;
}

esp_err_t twai_start(void) {
  if (!installed || state != TWAI_STATE_STOPPED) {
    return ESP_ERR_INVALID_STATE;
  }

  noInterrupts();
  rxHead = rxCount = 0;
  txHead = txCount = 0;
  interrupts();

  // Leaving init mode needs 11 recessive bits on RX. If it never happens,
  // the RX line is stuck dominant (no transceiver, wrong pin, shorted bus).
  CAN1->MCR &= ~CAN_MCR_INRQ;
  if (!waitMcrAck(false, 100)) {
    CAN1->MCR |= CAN_MCR_INRQ;
    return ESP_ERR_TIMEOUT;
  }

  noInterrupts();
  lastEsrFlags = CAN1->ESR & ESR_FLAGS;
  state = TWAI_STATE_RUNNING;
  interrupts();
  return ESP_OK;
}

esp_err_t twai_stop(void) {
  if (!installed || state != TWAI_STATE_RUNNING) {
    return ESP_ERR_INVALID_STATE;
  }

  noInterrupts();
  abortTransmissions();
  CAN1->MCR |= CAN_MCR_INRQ;
  state = TWAI_STATE_STOPPED;
  interrupts();

  waitMcrAck(true, 10);
  return ESP_OK;
}

esp_err_t twai_transmit(const twai_message_t *message, uint32_t timeout_ms) {
  if (!message) {
    return ESP_ERR_INVALID_ARG;
  }
  if (message->data_length_code > (message->dlc_non_comp ? 15 : TWAI_FRAME_MAX_DLC)) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }
  if (mode == TWAI_MODE_LISTEN_ONLY) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  uint32_t start = millis();
  for (;;) {
    pollErrorState();
    if (state != TWAI_STATE_RUNNING) {
      return ESP_ERR_INVALID_STATE;
    }

    NVIC_DisableIRQ(CAN1_TX_IRQn);
    if (txCount < txLen) {
      txQueue[(txHead + txCount) % txLen] = *message;
      txCount++;
      fillMailboxes();
      NVIC_EnableIRQ(CAN1_TX_IRQn);
      return ESP_OK;
    }
    NVIC_EnableIRQ(CAN1_TX_IRQn);

    if (millis() - start >= timeout_ms) {
      return ESP_ERR_TIMEOUT;
    }
    yield();
  }
}

esp_err_t twai_receive(twai_message_t *message, uint32_t timeout_ms) {
  if (!message) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }

  uint32_t start = millis();
  for (;;) {
    NVIC_DisableIRQ(CAN1_RX0_IRQn);
    if (rxCount > 0) {
      *message = rxQueue[rxHead];
      rxHead = (rxHead + 1) % rxLen;
      rxCount--;
      NVIC_EnableIRQ(CAN1_RX0_IRQn);
      return ESP_OK;
    }
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    if (millis() - start >= timeout_ms) {
      return ESP_ERR_TIMEOUT;
    }
    yield();
  }
}

esp_err_t twai_read_alerts(uint32_t *alerts, uint32_t timeout_ms) {
  if (!alerts) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }

  uint32_t start = millis();
  for (;;) {
    pollErrorState();

    noInterrupts();
    uint32_t pending = alertsPending;
    alertsPending = 0;
    interrupts();

    if (pending) {
      *alerts = pending;
      return ESP_OK;
    }
    if (millis() - start >= timeout_ms) {
      *alerts = 0;
      return ESP_ERR_TIMEOUT;
    }
    yield();
  }
}

esp_err_t twai_reconfigure_alerts(uint32_t alerts_enabled, uint32_t *current_alerts) {
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }

  noInterrupts();
  if (current_alerts) {
    *current_alerts = alertsPending;
  }
  alertsEnabled = alerts_enabled;
  alertsPending = 0;
  interrupts();
  return ESP_OK;
}

esp_err_t twai_initiate_recovery(void) {
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }
  pollErrorState();
  if (state != TWAI_STATE_BUS_OFF) {
    return ESP_ERR_INVALID_STATE;
  }

  // bxCAN (ABOM = 0): enter and leave init mode; the controller then waits
  // for 128 x 11 recessive bits and clears BOFF (→ TWAI_ALERT_BUS_RECOVERED).
  noInterrupts();
  state = TWAI_STATE_RECOVERING;
  raiseAlert(TWAI_ALERT_RECOVERY_IN_PROGRESS);
  interrupts();

  CAN1->MCR |= CAN_MCR_INRQ;
  waitMcrAck(true, 10);
  CAN1->MCR &= ~CAN_MCR_INRQ;
  return ESP_OK;
}

esp_err_t twai_get_status_info(twai_status_info_t *status_info) {
  if (!status_info) {
    return ESP_ERR_INVALID_ARG;
  }
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }

  pollErrorState();

  uint32_t esr = CAN1->ESR;
  uint32_t tsr = CAN1->TSR;
  uint32_t busyMailboxes = !(tsr & CAN_TSR_TME0) + !(tsr & CAN_TSR_TME1) + !(tsr & CAN_TSR_TME2);

  status_info->state            = state;
  status_info->msgs_to_tx       = txCount + busyMailboxes;
  status_info->msgs_to_rx       = rxCount;
  status_info->tx_error_counter = (esr & CAN_ESR_TEC) >> CAN_ESR_TEC_Pos;
  status_info->rx_error_counter = (esr & CAN_ESR_REC) >> CAN_ESR_REC_Pos;
  status_info->tx_success_count = txSuccessCount;
  status_info->tx_failed_count  = txFailedCount;
  status_info->rx_missed_count  = rxMissedCount;
  status_info->rx_overrun_count = rxOverrunCount;
  status_info->arb_lost_count   = arbLostCount;
  status_info->bus_error_count  = busErrorCount;
  return ESP_OK;
}

esp_err_t twai_clear_transmit_queue(void) {
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }
  noInterrupts();
  abortTransmissions();
  interrupts();
  return ESP_OK;
}

esp_err_t twai_clear_receive_queue(void) {
  if (!installed) {
    return ESP_ERR_INVALID_STATE;
  }
  NVIC_DisableIRQ(CAN1_RX0_IRQn);
  rxHead = rxCount = 0;
  NVIC_EnableIRQ(CAN1_RX0_IRQn);
  return ESP_OK;
}

const char *esp_err_to_name(esp_err_t err) {
  switch (err) {
    case ESP_OK:                return "ESP_OK";
    case ESP_FAIL:              return "ESP_FAIL";
    case ESP_ERR_INVALID_ARG:   return "ESP_ERR_INVALID_ARG";
    case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
    case ESP_ERR_NOT_SUPPORTED: return "ESP_ERR_NOT_SUPPORTED";
    case ESP_ERR_TIMEOUT:       return "ESP_ERR_TIMEOUT";
    default:                    return "UNKNOWN_ERROR";
  }
}
