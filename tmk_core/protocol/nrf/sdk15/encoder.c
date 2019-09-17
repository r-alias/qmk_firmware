#include "nrfx_config.h"
#include "nrf_drv_qdec.h"
#include "nrf_drv_rtc.h"
#include "nrf_gpio.h"
#include "nrf_drv_ppi.h"
#include "encoder.h"

// if you want to use event handler, USE_NRF_DRV_*** turn to 1.
#define USE_NRF_DRV_QDEC 0  // change to 1, setting QDEC via nrf_drv**()
// If USE_NRF_DRV_QDEC Change to 1. !!FIX SDK!! detail-> https://devzone.nordicsemi.com/f/nordic-q-a/37968/nrfx_qdec-driver-quirks

#define USE_NRF_DRV_RTC  0  // change to 1, setting RTC via nrf_drv**()

#define USE_QDEC_PPI 1      // change to 1, use ppi, to use qdec low power consumption. https://devzone.nordicsemi.com/f/nordic-q-a/35425/qdec-high-power-consumption
#define USE_NRF_DRV_PPI 1   // Setting PPI via NRF_DRV

#ifndef ENCODER_RESOLUTION
#    define ENCODER_RESOLUTION 1
#endif

#if !defined(ENCODERS_PAD_A) || !defined(ENCODERS_PAD_B)
#    error "No encoder pads defined by ENCODERS_PAD_A and ENCODERS_PAD_B"
#endif

#define NUMBER_OF_ENCODERS (sizeof(encoders_pad_a) / sizeof(pin_t))

__attribute__((weak)) void encoder_update_user(int8_t index, bool clockwise) {}

__attribute__((weak)) void encoder_update_kb(int8_t index, bool clockwise) { encoder_update_user(index, clockwise); }

const uint32_t encoders_pad_a[] = ENCODERS_PAD_A;
const uint32_t encoders_pad_b[] = ENCODERS_PAD_B;

#if USE_NRF_DRV_QDEC
static void qdec_event_handler(nrfx_qdec_event_t event) {}
#endif

#if USE_NRF_DRV_RTC
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
static void rtc_handler(nrf_drv_rtc_int_type_t int_type){}
#else
#define ENCODER_RTC NRF_RTC2
#endif

#if USE_NRF_DRV_PPI
static nrf_ppi_channel_t m_ppi_channel1;
static nrf_ppi_channel_t m_ppi_channel2;
#endif

void qdec_init (void) {
  static nrfx_qdec_config_t config = {
    .reportper          = NRF_QDEC_REPORTPER_DISABLED,
    .sampleper          = (nrf_qdec_sampleper_t)3,
    //.psela              = ,
    //.pselb              = ,
    .pselled            = 0xFFFFFFFF, // !!!!!! MUST FIX SDK when using nrfx_qdec_init(). ： https://devzone.nordicsemi.com/f/nordic-q-a/37968/nrfx_qdec-driver-quirks
    .ledpre             = NRFX_QDEC_CONFIG_LEDPRE,
    .ledpol             = (nrf_qdec_ledpol_t)NRFX_QDEC_CONFIG_LEDPOL,
    .interrupt_priority = NRFX_QDEC_CONFIG_IRQ_PRIORITY,
    .dbfen              = NRFX_QDEC_CONFIG_DBFEN,
    .sample_inten       = NRFX_QDEC_CONFIG_SAMPLE_INTEN
  };

  config.psela = encoders_pad_a[0];
  config.pselb = encoders_pad_b[0];

#if USE_NRF_DRV_QDEC
  uint32_t err_code;
  err_code = nrf_drv_qdec_init(&config, qdec_event_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_qdec_enable();
#else
  nrf_qdec_sampleper_set(config.sampleper);
  nrf_gpio_cfg_input(config.psela, NRF_GPIO_PIN_PULLUP);
  nrf_gpio_cfg_input(config.pselb, NRF_GPIO_PIN_PULLUP);
  nrf_qdec_pio_assign(config.psela, config.pselb, config.pselled);
  nrf_qdec_ledpre_set(config.ledpre);
  nrf_qdec_ledpol_set(config.ledpol);
  nrf_qdec_dbfen_enable();

  nrf_qdec_enable();
#endif
}

void rtc1_init(void) {

  nrf_drv_rtc_config_t config = {
    .prescaler          = RTC_FREQ_TO_PRESCALER(512),
    .interrupt_priority = NRFX_RTC_DEFAULT_CONFIG_IRQ_PRIORITY,
    .reliable           = NRFX_RTC_DEFAULT_CONFIG_RELIABLE,
    .tick_latency       = NRFX_RTC_US_TO_TICKS(NRFX_RTC_MAXIMUM_LATENCY_US, NRFX_RTC_DEFAULT_CONFIG_FREQUENCY),
  };

#if USE_NRF_DRV_RTC
  uint32_t err_code;

  //Initialize RTC instance
  err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
  APP_ERROR_CHECK(err_code);

  //Enable tick event & interrupt
  nrf_drv_rtc_tick_enable(&rtc,true);

  //Power on RTC instance
  nrf_drv_rtc_enable(&rtc);
#else
  nrf_rtc_prescaler_set(ENCODER_RTC, config.prescaler);
  nrf_rtc_event_enable(ENCODER_RTC, NRF_RTC_INT_TICK_MASK);
  nrf_rtc_task_trigger(ENCODER_RTC, NRF_RTC_TASK_START);
#endif
}

void encoder_init(void) {
  qdec_init();
  rtc1_init();

#if USE_QDEC_PPI
#if USE_NRF_DRV_PPI
  uint32_t err_code = NRF_SUCCESS;

  err_code = nrf_drv_ppi_init();
  APP_ERROR_CHECK(err_code);

  // channel 1

  err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel1);
  APP_ERROR_CHECK(err_code);

  uint32_t qdec_task_addr, qdec_event_addr, rtc_event_addr;
  nrf_drv_qdec_task_address_get(NRF_QDEC_TASK_START, &qdec_task_addr);

#if USE_NRF_DRV_RTC
  rtc_event_addr = nrf_drv_rtc_event_address_get(&rtc, NRF_RTC_EVENT_TICK);
#else
  rtc_event_addr = nrf_rtc_event_address_get(ENCODER_RTC, NRF_RTC_EVENT_TICK);
#endif

  err_code = nrf_drv_ppi_channel_assign(m_ppi_channel1,
                                        rtc_event_addr,
                                        qdec_task_addr);
  APP_ERROR_CHECK(err_code);

  // channel 2
  err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel2);
  APP_ERROR_CHECK(err_code);

  nrf_drv_qdec_task_address_get(NRF_QDEC_TASK_STOP, &qdec_task_addr);
  nrf_drv_qdec_event_address_get(NRF_QDEC_EVENT_SAMPLERDY, &qdec_event_addr);

  err_code = nrf_drv_ppi_channel_assign(m_ppi_channel2,
                                        qdec_event_addr,
                                        qdec_task_addr);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_drv_ppi_channel_enable(m_ppi_channel1);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_drv_ppi_channel_enable(m_ppi_channel2);
  APP_ERROR_CHECK(err_code);
#else
  //PPI connect RTC1 <-> QDEC.
 	NRF_PPI ->CH[0].EEP = (uint32_t)&(ENCODER_RTC->EVENTS_TICK);
	NRF_PPI ->CH[0].TEP = (uint32_t)&(NRF_QDEC->TASKS_START);
	NRF_PPI ->CHENSET|=1;

	NRF_PPI ->CH[1].EEP = (uint32_t)&(NRF_QDEC->EVENTS_SAMPLERDY);
	NRF_PPI ->CH[1].TEP = (uint32_t)&(NRF_QDEC->TASKS_STOP);
	NRF_PPI ->CHENSET|=2;

  //no qdec task trigger. Because qdec tasks will be started by RTC1.
#endif
#else
#if !USE_NRF_DRV_QDEC
  nrf_qdec_task_trigger(NRF_QDEC_TASK_START);
#endif
#endif

}


void encoder_read(void) {
  static int16_t encoder_value = 0;
  int16_t acc = 0;

#if USE_NRF_DRV_QDEC
  int16_t accdbl = 0;
  nrfx_qdec_accumulators_read(&acc, &accdbl);
#else
  nrf_qdec_task_trigger(NRF_QDEC_TASK_READCLRACC);
  acc = nrf_qdec_accread_get();
#endif

  encoder_value += acc;

  if(ENCODER_RESOLUTION <= encoder_value) {
    encoder_update_kb(0, false);
  } else if (encoder_value <= -ENCODER_RESOLUTION) {
    encoder_update_kb(0, true);
  }
  encoder_value %= ENCODER_RESOLUTION;
}
