/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "arm_math_types_f16.h"
#include "arm_math_f16.h"
#include "arm_const_structs_f16.h"
#include <math.h>
#include "mcp4725.h"
#include <stdlib.h>
#include "eeConfig.h"
#include "ee.h"
#include "baudot.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum commands_enum {
  CMD_NONE,
  CMD_READ_FIELDS,
  CMD_SET_FIELDS,
  CMD_TEST_COMMS
};

enum modes_enum {
  MODE_IDLE = 0,
  MODE_RX = 1,
  MODE_TX = 2,
  MODE_CALIBRATE_VCO = 3
};

enum fields_enum {
  FIELD_MODE,
  FIELD_FREQ_MHZ,
  FIELD_MARK_FREQ,
  FIELD_SPACE_FREQ,
  FIELD_BAUD_RATE,
  FIELD_TX_DATA,
  FIELD_RX_DATA_RDY,
  FIELD_RX_DATA,
  FIELD_RX_TONE,
  FIELD_VCO_DAC_VOLTAGE,
  FIELD_VCO_FREQ_CAL_VALUE,
  FIELD_PA_DAC_VOLTAGE,
  NUM_FIELDS
};

enum states_enum {
  STATE_GOTO_IDLE,
  STATE_IDLE,
  STATE_GOTO_RX,
  STATE_RX,
  STATE_GOTO_TX,
  STATE_TX,
  STATE_GOTO_CAL_VCO,
  STATE_CAL_VCO
};

enum rx_states_enum {
  RX_STATE_IDLE,
  RX_STATE_START,
  RX_STATE_READ_DATA,
  RX_STATE_STOP
};

enum tx_states_enum {
  TX_STATE_IDLE,
  TX_STATE_START,
  TX_STATE_SEND_DATA,
  TX_STATE_STOP
}; 

typedef struct state_struct {
  uint8_t state;
  uint64_t rx_begin_time;
  uint64_t tx_begin_time;
  uint8_t rx_symbol_state;
  uint8_t tx_symbol_state;
  uint8_t baudot_state;
} state_t;

typedef struct cal_point_struct {
  float32_t voltage;
  float32_t freq;
} cal_point_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIELD_SIZE      sizeof(uint32_t)
#define CMD_IND         0
#define BODY_LEN_IND    1
#define BODY_START_IND  2
#define TEST_COMMS_RPY  0xA5
#define FIELD_MEM_SIZE  FIELD_SIZE*NUM_FIELDS
#define SAMPLING_FREQ   48000.0
#define ADC_BUFFER_SIZE 512
#define FFT_SIZE        (ADC_BUFFER_SIZE/2)
#define WINDOW_FUNC_STDDEV  (ADC_BUFFER_SIZE/7.0)
#define WINDOW_FUNC_MEAN (ADC_BUFFER_SIZE/2.0)
#define SQRT_2_PI       2.506628
#define WINDOW_FUNC_COEF ((ADC_BUFFER_SIZE/2.718282)/(WINDOW_FUNC_STDDEV * SQRT_2_PI))
#define FFT_FREQ_RES    (SAMPLING_FREQ/ADC_BUFFER_SIZE)
#define DFLT_BAUD_RATE  45.45
#define FIRST_TX_IF_LO  4.0e6
#define TX_OFFSET_FREQ  28000
#define DFLT_MARK_FREQ  2125
#define DFLT_SPACE_FREQ (DFLT_MARK_FREQ + 170)
#define DFLT_FREQ_MHZ   28.08
#define DFLT_PA_DAC_VOLTAGE 0.5
#define VCO_DAC_ADDR    0xC0
#define PA_DAC_ADDR     0xC1
#define VCO_CAL_POINTS  512
#define CLOCK_FREQ      72.0e6
#define GET_TX_VCO_FREQ(carrier)  (carrier - (FIRST_TX_IF_LO + TX_OFFSET_FREQ))
#define DEBUG_BUF_SIZE  64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t fields[FIELD_MEM_SIZE];
uint8_t fields_checksum = 0;
extern uint8_t* UserRxBufferFS;
extern uint8_t* UserTxBufferFS;
state_t state;
volatile uint64_t microseconds = 0;
volatile uint32_t adc_reading;
volatile float16_t adc_samples_1[ADC_BUFFER_SIZE];
volatile float16_t adc_samples_2[ADC_BUFFER_SIZE];
volatile float16_t fft_output[ADC_BUFFER_SIZE];
volatile float16_t fft_mag_output[FFT_SIZE];
volatile float16_t* adc_samples = adc_samples_1;
volatile int sample_buf_selected = 0;
volatile int sample_ind = 0;
volatile bool sample_buf_rdy = false;
// arm_rfft_fast_instance_f32 var_inst_rfft_f32;
arm_rfft_fast_instance_f16 var_inst_rfft_f16;
volatile int rx_bit = -1;
volatile uint32_t rx_byte;
volatile int rx_bit_count;
volatile uint8_t tx_byte;
volatile uint8_t tx_bit_count;
mcp4725_t vco_dac;
mcp4725_t pa_dac;
cal_point_t vco_cal_pts[VCO_CAL_POINTS];
int vco_calpt_count = 0;
uint8_t debug_buf[DEBUG_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void init_fields(uint8_t* mem);
bool is_fields_updated(uint8_t* mem, uint8_t chksum);
bool parse_packet(uint8_t* packet_in, uint8_t* packet_out);
uint32_t get_field_u32(uint8_t* mem, uint8_t field_id);
void set_field_u32(uint8_t* mem, uint8_t field_id, uint32_t field_value);
float32_t get_field_f32(uint8_t* mem, uint8_t field_id);
void set_field_f32(uint8_t* mem, uint8_t field_id, float32_t field_value);
void set_field_f16(uint8_t* mem, uint8_t field_id, float16_t field_value);
void enable_rx();
void disable_rx();
void enable_tx();
void disable_tx();
uint64_t micros();
float32_t window_func(int index);
float32_t get_freq_from_index(int ind);
bool within_range(float32_t x, float32_t target, float32_t range);
void receive_symbol_state_machine(uint8_t* rx_symbol_state);
float32_t get_vco_dac_voltage(cal_point_t* points, float32_t freq);
void add_vco_cal_point(cal_point_t* points, float32_t voltage, float32_t freq);
int vco_cal_cmpfunc(const void* a, const void* b);
void save_vco_cal(cal_point_t* points);
void set_vco_freq_MHz(float32_t freq);
void transmit_symbol_state_machine(uint8_t* tx_state);
void set_audio_freq(float32_t freq);
void debug(char* str);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  init_fields(fields);

  // DACS
  vco_dac.addr = VCO_DAC_ADDR;
  vco_dac.hi2c = &hi2c1;
  vco_dac.vref = 3.3;

  pa_dac.addr = PA_DAC_ADDR;
  pa_dac.hi2c = &hi2c1;
  pa_dac.vref = 3.3;

  general_call_reset(&vco_dac);
  general_call_wakeup(&vco_dac);

  // SETUP STATE OBJECT 
  state.state = STATE_GOTO_IDLE;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USB COMMS */
    if(parse_packet(UserRxBufferFS, UserTxBufferFS)){
      CDC_Transmit_FS(UserTxBufferFS, UserTxBufferFS[BODY_LEN_IND] + BODY_START_IND);
    }

    /* BEGIN MAIN STATE MACHINE */
    switch(state.state){
      case STATE_GOTO_IDLE:{
        debug("Going to state IDLE...\r\n");
        disable_rx();
        disable_tx();
        state.state = STATE_IDLE;
        break;
      }
      case STATE_IDLE:{
        uint32_t mode = get_field_u32(fields, FIELD_MODE);
        if (mode = MODE_RX){
          state.state = STATE_GOTO_RX;
        } else if(mode == MODE_TX){
          state.state = STATE_GOTO_TX;
        } else if(mode == MODE_CALIBRATE_VCO){
          state.state = STATE_GOTO_CAL_VCO;
        }
        break;
      }
      case STATE_GOTO_RX:{
        debug("Going to state RX...\r\n");
        disable_tx();
        enable_rx();
        state.rx_begin_time = micros();
        state.state = STATE_RX;
        break;
      }
      case STATE_RX:{
        uint64_t baud_interval_us = (uint64_t)(1.0e6/get_field_f32(fields, FIELD_BAUD_RATE));
        if(sample_buf_rdy && micros() - state.rx_begin_time >= baud_interval_us){
          sample_buf_rdy = false;
          float32_t* samples;
          if(sample_buf_selected == 1){
            // switch adc_samples to point to adc_samples_2
            adc_samples = adc_samples_2;
            sample_buf_selected = 2;

            // now choose to process adc_samples_1
            samples = adc_samples_1;
          }else if(sample_buf_selected == 2){
            // switch adc_samples to point to adc_samples_1
            adc_samples = adc_samples_1;
            sample_buf_selected = 1;

            // now choose to process adc_samples_2
            samples = adc_samples_2;
          }

          state.rx_begin_time = micros();
          HAL_TIM_Base_Start(&htim3); //  restart ADC sampling

          /* BEGIN FFT ANALYSIS */
          arm_status status = arm_rfft_fast_init_f16(&var_inst_rfft_f16, ADC_BUFFER_SIZE); // must pass in the length of the real buffer size
          arm_rfft_fast_f16(&var_inst_rfft_f16, samples, fft_output, 0);
          arm_cmplx_mag_f16(fft_output, fft_mag_output, FFT_SIZE);
          float16_t max_value = 0;
          uint32_t test_ind = 0;
          arm_max_f16(fft_mag_output, FFT_SIZE, &max_value, &test_ind);
          float32_t freq = get_freq_from_index(test_ind);
          set_field_f32(fields, FIELD_RX_TONE, freq);
          /* END FFT ANALYSIS */

          if(within_range(freq, get_field_f32(fields, FIELD_MARK_FREQ), 1.25*FFT_FREQ_RES)){
            rx_bit = 1;
          }else if(within_range(freq, get_field_f32(fields, FIELD_SPACE_FREQ), 1.25*FFT_FREQ_RES)){
            rx_bit = 0;
          }else{
            rx_bit = -1;
          }

          if(rx_bit != -1){
            receive_symbol_state_machine(&state.rx_symbol_state);
            if(state.rx_symbol_state == RX_STATE_STOP){
              // copy the data
              set_field_u32(fields, FIELD_RX_DATA, rx_byte);
              set_field_u32(fields, FIELD_RX_DATA_RDY, 1);
            }
          }
        }

        if(get_field_u32(fields, FIELD_MODE) != MODE_RX){
          state.state = STATE_GOTO_IDLE;
        }

        break;
      }
      case STATE_GOTO_TX:{
        debug("Going to state TX...\r\n");
        disable_rx();
        enable_tx();
        state.state = STATE_TX;
        state.tx_begin_time = micros();
        set_vout(&pa_dac, get_field_f32(fields, FIELD_PA_DAC_VOLTAGE));
        break;
      }
      case STATE_TX:{
        if(get_field_u32(fields, FIELD_MODE) != MODE_TX){
          state.state = STATE_GOTO_IDLE;
        }else{
          uint64_t bit_int_us = (uint64_t)(1.0e6/get_field_f32(fields, FIELD_BAUD_RATE));
          // transmit whatever is in FIELD_TX_DATA at the specified baud
          if(micros() - state.tx_begin_time >= bit_int_us){
            state.tx_begin_time = micros();
            transmit_symbol_state_machine(&state.tx_symbol_state);
          }
        }
        break;
      }case STATE_GOTO_CAL_VCO:{
        debug("Going to state CALIBRATE VCO...\r\n");
        set_vout(&vco_dac, get_field_f32(fields, FIELD_VCO_DAC_VOLTAGE));
        state.state = STATE_CAL_VCO;
        break;
      }case STATE_CAL_VCO:{
        float32_t freq = get_field_f32(fields, FIELD_VCO_FREQ_CAL_VALUE);
        if(freq != 0.0){
          // if user has set the frequency, add this cal point
          float32_t volts = get_field_f32(fields, FIELD_VCO_DAC_VOLTAGE);
          add_vco_cal_point(vco_cal_pts, volts, freq);
          set_field_f32(fields, FIELD_VCO_FREQ_CAL_VALUE, 0.0);
          save_vco_cal(vco_cal_pts);
          set_field_u32(fields, FIELD_MODE, MODE_IDLE);
          state.state = STATE_IDLE;
        }
        break;
      }
    }
    /* END MAIN STATE MACHINE */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


void init_fields(uint8_t* mem){
  set_field_u32(mem, FIELD_MODE, (uint32_t)MODE_IDLE);
  set_field_f32(mem, FIELD_FREQ_MHZ, DFLT_FREQ_MHZ);
  set_field_f32(mem, FIELD_BAUD_RATE, DFLT_BAUD_RATE);
  set_field_f32(mem, FIELD_MARK_FREQ, DFLT_MARK_FREQ);
  set_field_f32(mem, FIELD_SPACE_FREQ, DFLT_SPACE_FREQ);
  set_field_f32(mem, FIELD_PA_DAC_VOLTAGE, DFLT_PA_DAC_VOLTAGE);

  uint8_t total = 0;
  for(int i = 0; i < NUM_FIELDS; i++){
    total += mem[i];
  }
  fields_checksum = (~total) + 1;

} 


bool is_fields_updated(uint8_t* mem, uint8_t chksum){
  uint8_t total = 0;
  for(int i = 0; i < NUM_FIELDS; i++){
    total += mem[i];
  }
  total += chksum;
  return total != 0;
}


/**
 * Packet format:
 *  [CMD] [BODY_LEN (N)] [BODY_0] ... [BODY_(N-1)]
*/
bool parse_packet(uint8_t* packet_in, uint8_t* packet_out){
  uint8_t cmd = packet_in[CMD_IND];
  uint8_t len = packet_in[BODY_LEN_IND];
  bool retval = false;

  if (cmd == CMD_TEST_COMMS){
    packet_out[CMD_IND] = CMD_TEST_COMMS;
    packet_out[BODY_LEN_IND] = 1;
    packet_out[BODY_START_IND] = TEST_COMMS_RPY;
    retval = true;
  }else if (cmd == CMD_READ_FIELDS){
    int ii = BODY_START_IND;
    for(int i = BODY_START_IND; i < len + BODY_START_IND; i++){
      packet_out[ii] = packet_in[i];
      ii++;
      memcpy(packet_out + ii, fields + (i*FIELD_SIZE), FIELD_SIZE);
      ii += FIELD_SIZE;
    }
    retval = true;
  }else if (cmd == CMD_SET_FIELDS){
    int i = BODY_START_IND;
    while(i < len + BODY_START_IND){
      int field_ind = packet_in[i];
      i++;
      memcpy(fields + field_ind*FIELD_SIZE, packet_in + i, FIELD_SIZE);
      i += FIELD_SIZE;
    }
    retval = true;
  }

  packet_in[CMD_IND] = CMD_NONE;
  return retval;
}


uint32_t get_field_u32(uint8_t* mem, uint8_t field_id){
  uint32_t retval = 0;
  memcpy((uint8_t*)&retval, mem + field_id*FIELD_SIZE, FIELD_SIZE);
  return retval;
}

void set_field_u32(uint8_t* mem, uint8_t field_id, uint32_t field_value){
  memcpy(mem + field_id*FIELD_SIZE, (uint8_t*)&field_value, FIELD_SIZE);
}

float32_t get_field_f32(uint8_t* mem, uint8_t field_id){
  float32_t retval = 0;
  memcpy((uint8_t*)&retval, mem + field_id*FIELD_SIZE, FIELD_SIZE);
  return retval;
}

void set_field_f32(uint8_t* mem, uint8_t field_id, float32_t field_value){
  memcpy(mem + field_id*FIELD_SIZE, (uint8_t*)&field_value, FIELD_SIZE);
}


void set_field_f16(uint8_t* mem, uint8_t field_id, float16_t field_value){
  float32_t temp = (float32_t)field_value;
  memcpy(mem + field_id*FIELD_SIZE, (uint8_t*)&temp, FIELD_SIZE);
}

/**
 * Enable receiver and related features
*/
void enable_rx(){
  HAL_GPIO_WritePin(LNA_EN_GPIO_Port, LNA_EN_Pin, 0);
  sample_buf_selected = 1;
  adc_samples = adc_samples_1;
  sample_ind = 0;
  HAL_ADC_Start_DMA(&hadc1, &adc_reading, 1);
  HAL_TIM_Base_Start(&htim3); // for 48kHz timebase and triggering ADC samples
  // TODO:
  //  set DAC voltage to program VCO for downconversion
  set_vco_freq_MHz(get_field_f32(fields, FIELD_FREQ_MHZ));
}

/**
 * Disable receiver and related features
*/
void disable_rx(){
  HAL_GPIO_WritePin(LNA_EN_GPIO_Port, LNA_EN_Pin, 0);
  HAL_TIM_Base_Stop(&htim3);
  HAL_ADC_Stop_DMA(&hadc1);
}

void enable_tx(){
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, 1);
  float32_t freq = get_field_f32(fields, FIELD_FREQ_MHZ) * 1.0e6;
  freq = GET_TX_VCO_FREQ(freq);
  set_vco_freq_MHz(freq/1.0e6);
}

void disable_tx(){
  HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, 0);
  HAL_TIM_Base_Stop(&htim1);
  HAL_TIM_Base_Stop(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM3){
    microseconds += 10;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  float32_t sample =  window_func(sample_ind) * (3.3*adc_reading)/4095.0;
  adc_samples[sample_ind] = sample;
  sample_ind++;
  if(sample_ind >= ADC_BUFFER_SIZE){
    sample_buf_rdy = true;

    // prevent the ADC from being triggered
    // we'll re-enable in sync with our baud rate timing
    HAL_TIM_Base_Stop(&htim3); 
  }
}
uint64_t micros(){
  return microseconds;
}

float32_t window_func(int index){
  return WINDOW_FUNC_COEF * expf(-0.5*powf((1.0*index - WINDOW_FUNC_MEAN)/WINDOW_FUNC_STDDEV, 2.0));
}

float32_t get_freq_from_index(int ind){
  return ind * (SAMPLING_FREQ/ADC_BUFFER_SIZE);
}


bool within_range(float32_t x, float32_t target, float32_t range){
  if(x < target){
    return x > target - range;
  }else if(x > target){
    return x < target + range;
  }else{
    return true;
  }
}


void receive_symbol_state_machine(uint8_t* rx_symbol_state){
  switch(*rx_symbol_state){
    case RX_STATE_IDLE:{
      // waiting for a start bit
      if(rx_bit == 0){
        // the received bit was a start bit
        // next bit will be data
        *rx_symbol_state = RX_STATE_READ_DATA;
        rx_bit_count = 0;
      }
      break;
    }case RX_STATE_READ_DATA:{
      // data comes in LSB first
      rx_byte |= (rx_bit << rx_bit_count);
      rx_bit_count++;
      if (rx_bit_count == 5){
        rx_bit_count = 0; // reset so we can use this for stop bit counting
        *rx_symbol_state = RX_STATE_STOP;
      }
      break;
    }case RX_STATE_STOP:{
      rx_bit_count++;
      if (rx_bit_count >= 1){
        *rx_symbol_state = RX_STATE_IDLE;
      }
      break;
    }
  }
}

float32_t get_vco_dac_voltage(cal_point_t* points, float32_t freq){
  
  cal_point_t* pt_a;
  cal_point_t* pt_b;
  int i = vco_calpt_count/2;
  int step_size = vco_calpt_count/4;
  bool found = false;

  while(!found){
    pt_a = points + i;
    pt_b = points + i + 1;

    if (freq < pt_a->freq){
      // look lower
      i -= step_size;
      step_size = (int)(step_size / 2.0 + 0.5);
    }else if(freq > pt_b->freq){
      // look higher
      i += step_size;
      step_size = (int)(step_size / 2.0 + 0.5);
    }else{
      found = true;
    }

    if (i + 1 >= vco_calpt_count){
      i -= 1;
      found = true;
    }else if(i < 0){
      i += 1;
      found = true;
    }
  }

  float32_t slope = (pt_b->freq - pt_a->freq)/(pt_b->voltage - pt_a->voltage);
  return ((freq - pt_a->freq)/slope) + pt_a->voltage;
}

void add_vco_cal_point(cal_point_t* points, float32_t voltage, float32_t freq){
  if(vco_calpt_count >= VCO_CAL_POINTS){
    return;
  }
  points[vco_calpt_count].voltage = voltage;
  points[vco_calpt_count].freq    = freq;
  vco_calpt_count++;
  qsort(points, vco_calpt_count, sizeof(cal_point_t), vco_cal_cmpfunc);
}

int vco_cal_cmpfunc(const void* a, const void* b){
  float32_t freq_a = ((cal_point_t *)a)->freq;
  float32_t freq_b = ((cal_point_t *)b)->freq;
  if (freq_a > freq_b){
    return 1;
  }else if (freq_b > freq_a){
    return -1;
  }else{
    return 0;
  }
}

void save_vco_cal(cal_point_t* points){

}


void set_vco_freq_MHz(float32_t freq_MHz){
  float32_t freq = 1.0e6 * freq_MHz;
  set_vout(&vco_dac, get_vco_dac_voltage(vco_cal_pts, freq));
}

void transmit_symbol_state_machine(uint8_t* tx_state){
  switch(*tx_state){
    case TX_STATE_IDLE:{
      set_audio_freq(get_field_f32(fields, FIELD_MARK_FREQ) + TX_OFFSET_FREQ);
      *tx_state = TX_STATE_START;
      break;
    }case TX_STATE_START:{
      set_audio_freq(get_field_f32(fields, FIELD_SPACE_FREQ) + TX_OFFSET_FREQ);
      tx_bit_count = 0;
      tx_byte = (uint8_t)(get_field_u32(fields, FIELD_TX_DATA) & 0xFF);
      *tx_state = TX_STATE_SEND_DATA;
      break;
    }case TX_STATE_SEND_DATA:{
      // TODO
      break;
    }
  }
}

/**
 * @brief sets the TIM2 channel 1 PWM output to the specified frequency
*/
void set_audio_freq(float32_t freq){
  float32_t counts = CLOCK_FREQ/freq;
  
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  HAL_TIM_Base_Stop(&htim2);
  if(counts <= 65535.0){
    /* Ensure the clock division is 1 */
    htim2.Instance->CR1 &= ~TIM_CR1_CKD;
    htim2.Instance->CR1 |= (uint32_t)TIM_CLOCKDIVISION_DIV1;
    htim2.Instance->ARR = (uint32_t)counts;
    htim2.Instance->CCR1 = counts / 2; // duty cycle of 0.5
  }else{
    counts = counts / 2;
    /* Set the clock division to be 2 */
    htim2.Instance->CR1 &= ~TIM_CR1_CKD;
    htim2.Instance->CR1 |= (uint32_t)TIM_CLOCKDIVISION_DIV2;
    htim2.Instance->ARR = (uint32_t)counts;
    htim2.Instance->CCR1 = counts / 2; // duty cycle of 0.5
  }
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}


void debug(char* str){
  HAL_UART_Transmit(&huart1, str, strlen(str), 10);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
