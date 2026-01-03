/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
// #include "task_init.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define CS1_High() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define CS1_Low() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS3_High() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define CS3_Low() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)

int spi_err = 0;

uint8_t debug1 = 0;
uint8_t debug2 = 0;

// Left corresponding to SPI3, while right side corresponding to SPI1

uint8_t motion_burst_data_R[12] = {0};
uint8_t motion_burst_data_L[12] = {0};
int16_t X_Speed_R, Y_Speed_R;
int16_t X_Speed_L, Y_Speed_L;
float X_R = 0;
float Y_R = 0;
float X_L = 0;
float Y_L = 0;

//flags
volatile uint8_t spi1_done = 0;
volatile uint8_t spi3_done = 0;
volatile uint8_t burst_flag = 0;

void delay_us(uint32_t us)
// max input: 1170(65535/168*3)
// Use Prescaler to make sure that max input is over 1000
{
  uint32_t start = TIM9->CNT;       
  uint32_t target_cycles = 168 * us / (2+1); // 168MHz, Prescaler = 2

  while (1)
  {
    uint32_t current = TIM9->CNT;
    if (current - start >= target_cycles)
      break;
    // current is in a next cycle which is ahead of start
    if (current < start)
    {
      if ((current + 65536 - start) >= target_cycles)
        break;
    }
  }
}

void delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

// SPI send and receive function(Full Duplex)
// return: [15:8](L)[7:0](R)
uint16_t SPI_SendReceive(uint8_t data)
{
  uint8_t rx_data1,rx_data3;
  uint16_t rx_data;
  HAL_SPI_TransmitReceive(&hspi1, &data, &rx_data1, 1, HAL_MAX_DELAY);
  HAL_SPI_TransmitReceive(&hspi3, &data, &rx_data3, 1, HAL_MAX_DELAY);
  rx_data = (uint16_t)(rx_data3 << 8);
  rx_data = rx_data | rx_data1;
  return rx_data;  
}

// SPI send and receive function for Burst Mode

void SPI_SendReceive_ForBurst(uint16_t* buffer)
{
  uint8_t rx_data1[12],rx_data3[12];
  uint8_t dummy_tx_data[12];
  memset(dummy_tx_data, 0x00, 12);
  HAL_SPI_TransmitReceive_DMA(&hspi1, dummy_tx_data, rx_data1, 12);
  HAL_SPI_TransmitReceive_DMA(&hspi3, dummy_tx_data, rx_data3, 12);
  // Using DMA and IT to ensure the simultaneity
  while(spi1_done != 1 || spi3_done != 1)
  {
    // wait for flags
  }
  spi1_done = 0; // reset flags
  spi3_done = 0;
  for(int itr = 0; itr < 12; itr++)
  {
    buffer[itr] = (uint16_t)(rx_data3[itr] << 8);
    buffer[itr] = buffer[itr] | rx_data1[itr];
  }
  return;  
}

// for debugging 
// to make sure how to shift
// not used in main function

int8_t final_shift = 0; // assumed as zero, but during the following debugging test, it would be set as -1 initially

void Debug_My_Shift_Logic(void) {
    uint8_t tx[3] = {0x00, 0xFF, 0xFF}; 
    uint8_t rx[3] = {0};
    
    // communucation test
    CS1_Low();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3, 100);
    CS1_High();

    // first set as zero during debugging
    final_shift = -1;

    for (int i = 0; i < 8; i++) {
        // transmit 0x00 and would always get 0x51 
        // traversal to see how many bits should be shifted
        uint8_t trial = (uint8_t)(rx[1] << i) | (rx[2] >> (8 - i));
        
        if (trial == 0x51) {
            final_shift = i; 
            break;
        }
    }
}

uint8_t tx_buf_R[3], rx_buf_R[3];
uint8_t tx_buf_L[3], rx_buf_L[3];

uint16_t read_register(uint8_t address)
{
  // result: [15:8](L)[7:0](R)  
  uint16_t result;
  uint8_t result_L;
  uint8_t result_R;

  CS1_Low(); // The start of Right Side Communication
  CS3_Low(); // The start of Left Side Communication

  delay_us(1);

  tx_buf_R[0] = address + 0x00; // transmit the address
  tx_buf_R[1] = 0xFF;           // dummy byte to receive
  tx_buf_R[2] = 0xFF;           // the second dummy byte

  tx_buf_L[0] = address + 0x00; // transmit the address
  tx_buf_L[1] = 0xFF;           // dummy byte to receive
  tx_buf_L[2] = 0xFF;           // the second dummy byte

  HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buf_L, rx_buf_L, 3);
  HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf_R, rx_buf_R, 3);

  while(spi1_done != 1 || spi3_done != 1)
  {

  } // wait for communication to be completed
  spi1_done = 0;
  spi3_done = 0;
  result_R = (rx_buf_R[1] << final_shift) | (rx_buf_R[2] >> (8 - final_shift)); // t_SRAD = 2 us
  result_L = (rx_buf_L[1] << final_shift) | (rx_buf_L[2] >> (8 - final_shift)); // t_SRAD = 2 us
 
  result = (uint16_t)result_L << 8;
  result = result | (uint16_t)result_R;
  CS1_High();
  CS3_High(); // end of read action
  delay_us(5); // t_swr = 5us
  return result;

  /* // potential solution to tSRAD = 2us without moving and montaging packs
  HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buf_L, rx_buf_L, 1);
  HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf_R, rx_buf_R, 1);
  while(spi1_done != 1 || spi3_done != 1)
  {

  } // wait for communication to be completed
  spi1_done = 0;
  spi3_done = 0;  
  delay_us(2); // tSRAD = 2us  
  HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buf_L+1, rx_buf_L+1, 1);
  HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf_R+1, rx_buf_R+1, 1);
  while(spi1_done != 1 || spi3_done != 1)
  {

  } // wait for communication to be completed
  spi1_done = 0;
  spi3_done = 0;  
  result_R =rx_buf_R[1];
  result_L =rx_buf_L[1];
  result = (uint16_t)result_L << 8;
  result = result | result_R;
  return result; */
}

void write_register(uint8_t address, uint8_t value)
{
  uint8_t tx_buf_L[2],tx_buf_R[2];
  uint8_t dummy_rx_buf_R[2],dummy_rx_buf_L[2];
  CS1_Low();
  CS3_Low();
  delay_us(1);
  tx_buf_L[0] = address + 0x80; // write operation itself needs MSB to be 1
  tx_buf_L[1] = value;          // data to be written into the designated register
  tx_buf_R[0] = address + 0x80; // write operation itself needs MSB to be 1
  tx_buf_R[1] = value;          // data to be written into the designated register
  HAL_SPI_TransmitReceive_DMA(&hspi1, tx_buf_R, dummy_rx_buf_R, 2);
  HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buf_L, dummy_rx_buf_L, 2);
  

  while(spi1_done != 1 || spi3_done != 1)
  {

  } // wait for communication to be completed
  spi1_done = 0;
  spi3_done = 0;
  CS1_High();    
  CS3_High();
  delay_us(5); 
}

static void Power_Up_Initializaton_Register_Setting(void)
{
  uint16_t read_tmp;
  uint8_t i;
  write_register(0x7F, 0x07);
  write_register(0x40, 0x41);
  write_register(0x7F, 0x00);
  write_register(0x40, 0x80);
  write_register(0x7F, 0x0E);
  write_register(0x55, 0x0D);
  write_register(0x56, 0x1B);
  write_register(0x57, 0xE8);
  write_register(0x58, 0xD5);
  write_register(0x7F, 0x14);
  write_register(0x42, 0xBC);
  write_register(0x43, 0x74);
  write_register(0x4B, 0x20);
  write_register(0x4D, 0x00);
  write_register(0x53, 0x0E);
  write_register(0x7F, 0x05);
  write_register(0x44, 0x04);
  write_register(0x4D, 0x06);
  write_register(0x51, 0x40);
  write_register(0x53, 0x40);
  write_register(0x55, 0xCA);
  write_register(0x5A, 0xE8);
  write_register(0x5B, 0xEA);
  write_register(0x61, 0x31);
  write_register(0x62, 0x64);
  write_register(0x6D, 0xB8);
  write_register(0x6E, 0x0F);

  write_register(0x70, 0x02);
  write_register(0x4A, 0x2A);
  write_register(0x60, 0x26);
  write_register(0x7F, 0x06);
  write_register(0x6D, 0x70);
  write_register(0x6E, 0x60);
  write_register(0x6F, 0x04);
  write_register(0x53, 0x02);
  write_register(0x55, 0x11);
  write_register(0x7A, 0x01);
  write_register(0x7D, 0x51);
  write_register(0x7F, 0x07);
  write_register(0x41, 0x10);
  write_register(0x42, 0x32);
  write_register(0x43, 0x00);
  write_register(0x7F, 0x08);
  write_register(0x71, 0x4F);
  write_register(0x7F, 0x09);
  write_register(0x62, 0x1F);
  write_register(0x63, 0x1F);
  write_register(0x65, 0x03);
  write_register(0x66, 0x03);
  write_register(0x67, 0x1F);
  write_register(0x68, 0x1F);
  write_register(0x69, 0x03);
  write_register(0x6A, 0x03);
  write_register(0x6C, 0x1F);

  write_register(0x6D, 0x1F);
  write_register(0x51, 0x04);
  write_register(0x53, 0x20);
  write_register(0x54, 0x20);
  write_register(0x71, 0x0C);
  write_register(0x72, 0x07);
  write_register(0x73, 0x07);
  write_register(0x7F, 0x0A);
  write_register(0x4A, 0x14);
  write_register(0x4C, 0x14);
  write_register(0x55, 0x19);
  write_register(0x7F, 0x14);
  write_register(0x4B, 0x30);
  write_register(0x4C, 0x03);
  write_register(0x61, 0x0B);
  write_register(0x62, 0x0A);
  write_register(0x63, 0x02);
  write_register(0x7F, 0x15);
  write_register(0x4C, 0x02);
  write_register(0x56, 0x02);
  write_register(0x41, 0x91);
  write_register(0x4D, 0x0A);
  write_register(0x7F, 0x0C);
  write_register(0x4A, 0x10);
  write_register(0x4B, 0x0C);
  write_register(0x4C, 0x40);
  write_register(0x41, 0x25);
  write_register(0x55, 0x18);
  write_register(0x56, 0x14);
  write_register(0x49, 0x0A);
  write_register(0x42, 0x00);
  write_register(0x43, 0x2D);
  write_register(0x44, 0x0C);
  write_register(0x54, 0x1A);
  write_register(0x5A, 0x0D);
  write_register(0x5F, 0x1E);
  write_register(0x5B, 0x05);
  write_register(0x5E, 0x0F);
  write_register(0x7F, 0x0D);
  write_register(0x48, 0xDD);
  write_register(0x4F, 0x03);
  write_register(0x52, 0x49);

  write_register(0x51, 0x00);
  write_register(0x54, 0x5B);
  write_register(0x53, 0x00);

  write_register(0x56, 0x64);
  write_register(0x55, 0x00);
  write_register(0x58, 0xA5);
  write_register(0x57, 0x02);
  write_register(0x5A, 0x29);
  write_register(0x5B, 0x47);
  write_register(0x5C, 0x81);
  write_register(0x5D, 0x40);
  write_register(0x71, 0xDC);
  write_register(0x70, 0x07);
  write_register(0x73, 0x00);
  write_register(0x72, 0x08);
  write_register(0x75, 0xDC);
  write_register(0x74, 0x07);
  write_register(0x77, 0x00);
  write_register(0x76, 0x08);
  write_register(0x7F, 0x10);
  write_register(0x4C, 0xD0);
  write_register(0x7F, 0x00);
  write_register(0x4F, 0x63);
  write_register(0x4E, 0x00);
  write_register(0x52, 0x63);
  write_register(0x51, 0x00);
  write_register(0x54, 0x54);
  write_register(0x5A, 0x10);
  write_register(0x77, 0x4F);
  write_register(0x47, 0x01);
  write_register(0x5B, 0x40);
  write_register(0x64, 0x60);
  write_register(0x65, 0x06);
  write_register(0x66, 0x13);
  write_register(0x67, 0x0F);
  write_register(0x78, 0x01);
  write_register(0x79, 0x9C);
  write_register(0x40, 0x00);
  write_register(0x55, 0x02);
  write_register(0x23, 0x70);
  write_register(0x22, 0x01);

  // Wait for 1ms
  delay_ms(1);

  for (i = 0; i < 60; i++)
  {
    read_tmp = read_register(0x6C);
    if (read_tmp == 0x8080)
      break;
    delay_us(1000);
  }
  if (i == 60)
  {
    write_register(0x7F, 0x14);
    write_register(0x6C, 0x00);
    write_register(0x7F, 0x00);
  }

  write_register(0x22, 0x00);
  write_register(0x55, 0x00);
  write_register(0x7F, 0x07);
  write_register(0x40, 0x40);
  write_register(0x7F, 0x00);
}

void PAW3395_Init(void)
{
  uint8_t reg_it;

  delay_ms(150);

  CS1_High();
  CS3_High();
  delay_ms(1);
  CS1_Low();
  CS3_Low();
  delay_ms(1);
  CS1_High();
  CS3_High();
  delay_ms(1);

  // write_register(0x3A, 0x5A);

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10); 

  delay_ms(10);

  Power_Up_Initializaton_Register_Setting();

  delay_ms(1);

  for (reg_it = 0x02; reg_it <= 0x06; reg_it++)
  {
    read_register(reg_it);
    delay_ms(3);
  }

  delay_ms(1);
}

uint16_t buffer[12];

void Motion_Burst(uint8_t *buffer_L,uint8_t *buffer_R)
{
  CS1_Low();
  CS3_Low();
  delay_us(1);
  // Start Communication
  // Send Motion_Brust address(0x16)
  SPI_SendReceive(0x16); 
  // Wait for tSRAD  
  delay_us(2);
  // Start reading SPI data continuously up to 12 bytes.
  SPI_SendReceive_ForBurst(buffer); 
 
  CS1_High();
  CS3_High(); 
  delay_us(5);
  for(int i = 0; i < 12; i++)
  {
    buffer_L[i] = (uint8_t)(buffer[i] >> 8); // left:[15:8]
    buffer_R[i] = (uint8_t)(buffer[i]); // right:[7:0]
  }   
  //tSRAD has already been considered so theoretically there's no need to move and montage
 /*  for (int i = 0; i < 11; i++)
  {
    buffer_L[i] = ((buffer_L[i] & 0x7F) << 1) | ((buffer_L[i + 1] & 0x80) >> 7);
    buffer_R[i] = ((buffer_R[i] & 0x7F) << 1) | ((buffer_R[i + 1] & 0x80) >> 7);
  }
 */
  X_Speed_L = (int16_t)(buffer_L[2] + (buffer_L[3] << 8));
  Y_Speed_L = (int16_t)(buffer_L[4] + (buffer_L[5] << 8));

  X_Speed_R = (int16_t)(buffer_R[2] + (buffer_R[3] << 8));
  Y_Speed_R = (int16_t)(buffer_R[4] + (buffer_R[5] << 8));  

  // resolution
  X_L += X_Speed_L / 5000.0 * 25.4;
  Y_L += Y_Speed_L / 5000.0 * 25.4;  
  X_R += X_Speed_R / 5000.0 * 25.4;
  Y_R += Y_Speed_R / 5000.0 * 25.4;
}

uint8_t debug1_t = 0;
uint8_t debug2_t = 0;
uint8_t debug3_t = 0;
uint8_t debug4_t = 0;

static CAN_RxHeaderTypeDef rx_header;
static CAN_TxHeaderTypeDef tx_header;

uint8_t can1TxData1[8] = {0};
uint8_t can1TxData2[8] = {0};
static uint8_t can1RxData[8] = {0};

void CanFilter_Init(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef canfilter;

  canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

  // filtrate any ID you want here
  canfilter.FilterIdHigh = 0x0000;
  canfilter.FilterIdLow = 0x0000;
  canfilter.FilterMaskIdHigh = 0x0000;
  canfilter.FilterMaskIdLow = 0x0000;

  canfilter.FilterActivation = ENABLE;
  canfilter.SlaveStartFilterBank = 14;

  // use different filter for can1&can2

  canfilter.FilterBank = 14;
  canfilter.FilterFIFOAssignment = CAN_FilterFIFO1;

  if (HAL_CAN_ConfigFilter(hcan, &canfilter) != HAL_OK)
  {
    Error_Handler();
  }
}

int can_error = 0;

void sendCan(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t tx_data[8])
{
  uint32_t mail_box = 0;

  tx_header.StdId = id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;

  if (HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &mail_box) != HAL_OK)
  {
    can_error++;
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan == &hcan2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, can1RxData) == HAL_OK)
    {
      // if (rx_header.StdId == 0x1FF)
      // {
      //   // dribbler_mode = canRxData[2];
      //   HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      // }
    }
  }
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

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
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
 
  HAL_TIM_Base_Start(&htim9);      
  __HAL_TIM_SET_COUNTER(&htim9, 0); 
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  CanFilter_Init(&hcan2);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  PAW3395_Init();
  Debug_My_Shift_Logic(); // For debug to see how to shift

  // write_register(0x5B,0x60);
  debug1_t = read_register(0x48);
  debug2_t = read_register(0x49);
  debug3_t = read_register(0x4A);
  debug4_t = read_register(0x4B);

  HAL_TIM_Base_Start_IT(&htim3);

  while (1)
  {
    if(burst_flag)
    {
      burst_flag = 0;
      Motion_Burst(motion_burst_data_L,motion_burst_data_R); 
      memcpy(can1TxData1, &X_L, sizeof(float));
      memcpy(can1TxData1+4, &Y_L, sizeof(float));
      sendCan(&hcan2, 0x300, can1TxData1);
      memcpy(can1TxData2, &X_R, sizeof(float));
      memcpy(can1TxData2+4, &Y_R, sizeof(float));
      sendCan(&hcan2, 0x301, can1TxData2);
    }    
 }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
 
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        spi1_done = 1;
    }
    if (hspi == &hspi3) {
        spi3_done = 1;
    }
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
