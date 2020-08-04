/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include "ldl_radio.h"
#include "ldl_mac.h"
#include "ldl_sm.h"
#include "ldl_system.h"
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
I2C_HandleTypeDef hi2c1;

LPTIM_HandleTypeDef hlptim1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

uint8_t temp;
uint8_t transmit=0;
uint8_t reciev=0;
uint8_t Cycle1;

uint32_t test1;
HAL_StatusTypeDef status;
const char *TAG = "main";
uint16_t Lpcounter;

//NOTEA Variable des paramatres environnementaux

uint8_t ENV[9];

I2C_HandleTypeDef restI2C;

SPI_HandleTypeDef resetHspi1;
uint8_t adresses[] = {0x0F ,0x10 ,0x20 ,0x21 ,
		0x22 ,0x23 ,0x24 ,0x25 ,0x27 ,0x28 ,0x29 ,0x2A
		,0x30 ,0x31 ,0x08 ,0x09 ,0x0A};

uint8_t register_[18];




/////////////////////LDL_Variables

struct ldl_radio radio;
struct ldl_mac mac;
struct ldl_sm sm;


const uint8_t device_eui [] = {
		0x00, 0xFC, 0xEC, 0x2E, 0xCB, 0x78, 0x2D, 0x2B
};

const uint8_t application_eui [] = {
		 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0x29, 0x9D
};

const uint8_t application_key [] = {

		0x20, 0xB5, 0xDF, 0x56, 0xBA, 0xB0, 0x33, 0x90, 0x38, 0xFA, 0xCF, 0xCD, 0xDB, 0x5D, 0xDA, 0xBF
};

const uint8_t nwk_key_ptr [] = {
		0x20, 0xB5, 0xDF, 0x56, 0xBA, 0xB0, 0x33, 0x90, 0x38, 0xFA, 0xCF, 0xCD, 0xDB, 0x5D, 0xDA, 0xBF
};




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

////
//printf debug

///////////


uint8_t Sensor_IO_Read( void *handle,
						uint8_t SensorAddr,
						uint8_t ReadAddr,
						uint8_t *pBuffer,
						uint16_t nBytesToRead );

uint8_t Sensor_IO_Write( void *handle,
						 uint8_t SensorAddr,
						 uint8_t WriteAddr,
						 uint8_t *pBuffer,
						 uint16_t nBytesToWrite );

void LPS25HB_HTS221_Set_Config(void *handle);

uint32_t LPS25HB_Pressure_Read( void * handle);

void HTS221_Get_Temperature_Humidity(void *handle,
									 int16_t *temperature,
									 uint16_t *humidity);
void Execute_cycle(void);

//lora stuff
void app_handler(void *app,
				 enum ldl_mac_response_type type,
				 const union ldl_mac_response_arg *arg);


//UNCOM dont forget this
void enable_interrupts(void);

uint32_t GetTimer();
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
//we send empty packets at first
//then we configure
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
  MX_I2C1_Init();
  MX_LPTIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  //PAST HERE
  initialize();
  printf("hello world");

//Initialize security module
  LDL_SM_init(&sm, application_key, nwk_key_ptr);
  LDL_INFO(TAG,"SM INIT");

//Initialise the radio

  LDL_Radio_init(&radio, LDL_RADIO_SX1272, NULL);
  LDL_INFO(TAG,"RADIO INIT");
  /* This radio has two power amplifiers. The amplifier in use
   * depends on the hardware (i.e. which pin the PCB traces connect).
   *
   * You have to tell the driver which amplifier is connected:
   *
   * - The Semtech MBED SX1272 shield uses LDL_RADIO_PA_RFO
   * - The HopeRF RFM95 SX1276 module uses LDL_RADIO_PA_BOOST
   *
   * */
  LDL_Radio_setPA(&radio, LDL_RADIO_PA_RFO);
  LDL_INFO(TAG,"RADIO PA INIT")


//Initialise The MAC LAYER
  struct ldl_mac_init_arg arg = {0};

  arg.radio = &radio;
  arg.handler = app_handler;
  arg.app = NULL;
  arg.sm = &sm;
  arg.joinEUI = application_eui;
  arg.devEUI = device_eui;

  LDL_MAC_init(&mac, LDL_EU_863_870, &arg);
  LDL_INFO(TAG,"MAC INIT");

  enable_interrupts();
  LDL_INFO(TAG,"INTERRUPT ENABLE");




  HAL_LPTIM_Counter_Start_IT(&hlptim1, 0xffff);
  //toDelete
//  for(;;){
//  HAL_SPI_TransmitReceive(&hspi1, &transmit, &reciev, 1, 0xfff);
//  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
//  transmit++;
//  HAL_Delay(1000);
//
//  }
  /* USER CODE END 2 */
test1=(LDL_System_tps()) << 4U;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(LDL_MAC_ready(&mac)){
		  LDL_INFO(TAG,"MAC READY");
	             if(LDL_MAC_joined(&mac)){
	            	 LDL_INFO(TAG,"JOINED ...");
	                 LDL_MAC_unconfirmedData(&mac, 1U, NULL, 0U, NULL);
	                 LDL_INFO(TAG,"UNCONFIRMED DATA");
	             }
	             else{

	                 LDL_MAC_otaa(&mac);
	                 LDL_INFO(TAG,"OTAA");
	             }
	         }
	  LDL_MAC_process(&mac);
//uint32_t waw;
//waw=(LDL_MAC_ticksUntilNextEvent(&mac) /(LDL_System_tps()/1000));
//	  HAL_Delay(waw);

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler(__func__);
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler(__func__);
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler(__func__);
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler(__func__);
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler(__func__);
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler(__func__);
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV2;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler(__func__);
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler(__func__);
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : Reset_Pin */
  GPIO_InitStruct.Pin = Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIO1_Pin DIO3_Pin DIO2_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin|DIO3_Pin|DIO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);



}

/* USER CODE BEGIN 4 */


//Projects stuff





uint8_t Sensor_IO_Write( void *handle,uint8_t SensorAddr, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite ){

	//LDL_SYSTEM_ENTER_CRITICAL(void)
	if ( nBytesToWrite > 1 ) WriteAddr |= 0x80;  /* Enable I2C multi-bytes Write */
	status=HAL_I2C_Mem_Write(handle,SensorAddr,WriteAddr,1,pBuffer,nBytesToWrite,0xff);

//	LDL_SYSTEM_LEAVE_CRITICAL(void)


	if(status!=HAL_OK){
		Error_Handler(__func__);
		  return 1;
	  }
	  return 0;
}
uint8_t Sensor_IO_Read( void *handle,uint8_t SensorAddr, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead ){
	//LDL_SYSTEM_ENTER_CRITICAL(void)

	if ( nBytesToRead > 1 ) ReadAddr |= 0x80; /* Enable I2C multi-bytes Read */
	status = HAL_I2C_Mem_Read(handle,SensorAddr,ReadAddr,1,pBuffer,nBytesToRead,0xff);
	//LDL_SYSTEM_LEAVE_CRITICAL(void)

	if(status!=HAL_OK){
		Error_Handler(__func__);
		  return 1;
	  }
	  return 0;
}




void LPS25HB_HTS221_Set_Config(void *handle)
{

	uint8_t Conf_reg1;
	uint8_t Conf_reg2;
	uint8_t Conf_reg3;
	uint8_t Interrupt_conf;
	uint8_t Threshold_L=(uint8_t)(16 * THRESHOLD);
	uint8_t Threshold_H=(uint8_t)(((uint16_t)(16 * THRESHOLD)) >> 8);
///////////////////////resetAZ

Conf_reg1 =  (LPS25HB_CTRL_REG1_PD | LPS25HB_CTRL_REG1_ODR_1HZ );
				status |= HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG1,1,&Conf_reg1,1,0xff);


Conf_reg2 = (LPS25HB_CTRL_REG2_AUTOZERO);
	 	 	 status |=HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG2,1,&Conf_reg2,1,0xff);


Conf_reg3=(LPS25HB_CTRL_REG3_INT_S_Pressure_LH );
	 	 	 	 status |=HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG3,1,&Conf_reg3,1,0xff);

Interrupt_conf= (LPS25HB_INTERRUPT_CFG_LIR );
status |=HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_INTERRUPT_CFG,1,&Interrupt_conf,1,0xff);

status |=HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_THS_P_L,1,&Threshold_L,1,0xff);

status |=HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_THS_P_H,1,&Threshold_H,1,0xff);

status |=HAL_I2C_Mem_Read(&hi2c1,LPS25HB_ADDRESS_READ,LPS25HB_INTERRUPT_CFG,1,&Interrupt_conf,1,0xff);

Interrupt_conf|=(LPS25HB_INTERRUPT_CFG_PL_E | LPS25HB_INTERRUPT_CFG_PH_E);

status |=HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_INTERRUPT_CFG,1,&Interrupt_conf,1,0xff);

status |=HAL_I2C_Mem_Read(&hi2c1,LPS25HB_ADDRESS_READ,LPS25HB_CTRL_REG1,1,&Conf_reg1,1,0xff);


Conf_reg1 |= (LPS25HB_CTRL_REG1_DIF_E );

status |=HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG1,1,&Conf_reg1,1,0xff);

//status |=HAL_I2C_Mem_Read(&hi2c1,LPS25HB_ADDRESS_READ,LPS25HB_CTRL_REG2,1,&Conf_reg2,1,0xff);

///////////////////////////////////////

				uint8_t buffer[2];

				  Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_AV_CONF_REG, buffer, 1);

				  buffer[0] &= ~HTS221_AVGT ;
				  buffer[0] &= ~HTS221_AVGH ;

				  Sensor_IO_Write(handle, HTS221_ADDRESS_WRITE, HTS221_AV_CONF_REG, &buffer[0], 1);

				  Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_CTRL_REG1, buffer, 2);

				  buffer[0] &= 0x78;

				  buffer[0] |= (HTS221_ODR | HTS221_BDU | HTS221_PD);

				  buffer[1] &= 0x7C;

				  Sensor_IO_Write(handle, HTS221_ADDRESS_WRITE, HTS221_CTRL_REG1, buffer, 2);

if(status!=HAL_OK){
	Error_Handler(__func__);
}
				  /* EXTI interrupt init*/
				  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
				  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

///////////////////////////
//read value


uint32_t LPS25HB_Pressure_Read( void * handle){

	//Local Variables
	uint8_t Buffer[3];
	uint8_t Buffer1[3];
	int32_t Raw_Pressure = 0;
	int32_t Ref_Pressure = 0;

/*	do{
		HAL_I2C_Mem_Read(&hi2c1,LPS25HB_ADDRESS_READ,LPS25HB_STATUS_REG,1,&STATUS,1,0xff);
	}while((STATUS & LPS25HB_STATUS_REG_PDA) == 0);*/
	//Read the pressure output registers
	Sensor_IO_Read(handle, LPS25HB_ADDRESS_READ, LPS25HB_PRESS_OUT_XL, Buffer, 3);

	Sensor_IO_Read(handle, LPS25HB_ADDRESS_READ, LPS25HB_PRESS_REF_XL, Buffer1, 3);

	//TRANSFOREME TO RAW PRESSURE
	Raw_Pressure = ((Buffer1[2] << 16) | (Buffer[1] << 8) | Buffer[0]);
	Ref_Pressure = ((Buffer1[2] << 16) | (Buffer1[1] << 8) | Buffer1[0]);


	//convert the 2's complement 24 bit to 2's complement 32 bit
	if (Raw_Pressure & 0x00800000)    // negative number
		{
		//convert to positive number
		Raw_Pressure |= 0xFF000000;
	  }
	if (Ref_Pressure & 0x00800000)    // negative number
		{
		//convert to positive number
		Ref_Pressure |= 0xFF000000;
	  }
	//Calculate Pressure in mbar
	return (Raw_Pressure+Ref_Pressure);
}

void HTS221_Get_Temperature_Humidity(void *handle, int16_t *temperature,uint16_t *humidity)
{
 int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
 int16_t T0_degC, T1_degC;
 uint8_t buffer[4], tmp;
 float   tmp_f;


Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_T0_DEGC_X8, buffer, 2);

Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_T0_T1_DEGC_H2, &tmp, 1);

 T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)buffer[0]);
 T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)buffer[1]);
 T0_degC = T0_degC_x8_u16 >> 3;
 T1_degC = T1_degC_x8_u16 >> 3;


Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_T0_OUT_L, buffer, 4);


 T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];
 T1_out = (((uint16_t)buffer[3]) << 8) | (uint16_t)buffer[2];


Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_TEMP_OUT_L_REG, buffer, 2);


 T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

 tmp_f = (float)(T_out - T0_out) * (float)(T1_degC - T0_degC) / (float)(T1_out - T0_out)  +  T0_degC;
 tmp_f *= 10.0f;

 *temperature = ( int16_t )tmp_f;
 ////////////////////////////////////////////////

 int16_t H0_T0_out, H1_T0_out, H_T_out;
 int16_t H0_rh, H1_rh;
 tmp_f = buffer[0] = buffer[1] = buffer[2] =0;;


Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_H0_RH_X2, buffer, 2);

 H0_rh = buffer[0] >> 1;
 H1_rh = buffer[1] >> 1;
Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_H0_T0_OUT_L, buffer, 2);

 H0_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_H1_T0_OUT_L, buffer, 2);

 H1_T0_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

Sensor_IO_Read(handle, HTS221_ADDRESS_READ, HTS221_HR_OUT_L_REG, buffer, 2);

 H_T_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

 tmp_f = (float)(H_T_out - H0_T0_out) * (float)(H1_rh - H0_rh) / (float)(H1_T0_out - H0_T0_out)  +  H0_rh;
 tmp_f *= 10.0f;

 *humidity = ( tmp_f > 1000.0f ) ? 1000
          : ( tmp_f <    0.0f ) ?    0
          : ( uint16_t )tmp_f;


}


////////////////////////////////

void Execute_cycle(void)
{
	//cycle captage
	if (Cycle1==0) {

		SET_BIT(PWR->CR, (PWR_CR_LPRUN ));

		MX_I2C1_Init();
	 	HAL_ResumeTick();

	//	for (int i = 0; i < 17; ++i) {
	//		 HAL_I2C_Mem_Read(&hi2c1,LPS25HB_ADDRESS_READ,adresses[i],1,&register_[i],1,0xff);
	//	 }

	 	//Read sensors value
	 	uint32_t temp;
	 	uint16_t temp2;
	 	uint16_t temp3;

	 	temp =LPS25HB_Pressure_Read(&hi2c1);
	 	ENV[0]=(uint8_t)temp;
	 	ENV[1]=(uint8_t)(temp>>8);
	 	ENV[2]=(uint8_t)(temp>>16);

	 	HTS221_Get_Temperature_Humidity(&hi2c1, &temp2, &temp3);

		ENV[3]=(uint8_t)temp2;
	 	ENV[4]=(uint8_t)(temp3>>8);
	 	ENV[5]=(uint8_t)(temp2);
		ENV[6]=(uint8_t)(temp3>>8);

		//set the new pressure in the pressure refrence regster
		{
			uint8_t temp=0;
			status|= HAL_I2C_Mem_Read(&hi2c1,LPS25HB_ADDRESS_READ,LPS25HB_INTERRUPT_Source,1,&temp,1,0xff);
			 temp=0;

			temp =  ( LPS25HB_CTRL_REG1_ODR_1HZ );
			status|= HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG1,1,&temp,1,0xff);


			temp =  ( LPS25HB_CTRL_REG1_ODR_1HZ | LPS25HB_CTRL_REG1_RESET_AZ);
			status|= HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG1,1,&temp,1,0xff);

			temp = (LPS25HB_CTRL_REG2_AUTOZERO);
			status |=HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG2,1,&temp,1,0xff);


			temp =  (LPS25HB_CTRL_REG1_PD | LPS25HB_CTRL_REG1_ODR_25HZ );
			status|= HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG1,1,&temp,1,0xff);

			HAL_Delay(50);

			temp =  (LPS25HB_CTRL_REG1_PD | LPS25HB_CTRL_REG1_ODR_1HZ | LPS25HB_CTRL_REG1_DIF_E);
			status|= HAL_I2C_Mem_Write(&hi2c1,LPS25HB_ADDRESS_WRITE,LPS25HB_CTRL_REG1,1,&temp,1,0xff);


		}
		HAL_I2C_MspDeInit(&hi2c1);
		//SystemClock_Config(RCC_MSIRANGE_0);
		CLEAR_BIT(PWR->CR, (PWR_CR_LPRUN ));

//		MX_ADC_Init();
//		HAL_ADC_Start(&hadc);
//		HAL_ADC_PollForConversion(&hadc, 0xff);
//		temp=HAL_ADC_GetValue(&hadc);

		ENV[7]=(uint8_t)temp;
	 	ENV[8]=(uint8_t)(temp>>8);


		//Deinit perepherals
	 	HAL_SuspendTick();
//		HAL_ADC_DeInit(&hadc);
		hi2c1 =restI2C;

		Cycle1=1;

	} else {

		MX_SPI1_Init();

		//send data through spi
/*		uint8_t AdressH,AdressL=0;
		AdressH =0x08;
		AdressL=0x64;
		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);//set CS =1

		HAL_SPI_Transmit(&hspi1,&AdressH,1,HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi1,&AdressL,1,HAL_MAX_DELAY);

		HAL_SPI_Transmit(&hspi1,ENV,9,HAL_MAX_DELAY);

		HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);// Reset SC=0

		HAL_SPI_MspDeInit(&hspi1);
		hspi1=resetHspi1;*/

		Cycle1=0;
	}

}
















//UNCOM : LORA Stuff


//Enable Interrupts after MAC initialisation

///////////////////////LORA STUFF
// dont forget to set the nss pin to 1 if refreshed project
////////////////////




//UNCOM get system time (ticks)
uint32_t LDL_System_ticks(void *app)
{
    return GetTimer(); // Provide a tick value in millisecond.
}

//UNCOM ticks per second
uint32_t LDL_System_tps(void)
{
    return 16384UL;  // Return tick frequency.
}

// UNCOM error per second
uint32_t LDL_System_eps(void)
{

    return 0UL;
}


//////////////////////// email contact@cjh.id.au



uint16_t SPI_InOut(uint16_t txData)
{
  uint16_t rxData ;
  HAL_StatusTypeDef status;
  status =HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) &txData, (uint8_t *) &rxData, 1, HAL_MAX_DELAY);
  if(status!=HAL_OK)LDL_ERROR(TAG,"Error SPI");
  return rxData;
}

//UNCOM chip write
void LDL_Chip_write(void *self, uint8_t addr, const void *data, uint8_t size)
{
    /* unused in this example */
    (void)self;

    const uint8_t *ptr = (uint8_t *)data;
    uint8_t i;
    //NSS = 0;
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );

    {
        /* SX1272/6 set the MSb of address indicate write */
    	SPI_InOut(addr |0x80);

 //   	    LDL_INFO("SPI Transmit %d",addr)
    	  for(i=0; i < size; i++){
    		  SPI_InOut(ptr[i]);
 //       	LDL_INFO("data transmiting %d ",ptr[i])
    	  }
    }
    //NSS = 1;
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
}


//UNCOM chip Read
void LDL_Chip_read(void *self, uint8_t addr, void *data, uint8_t size)
{
    /* unused in this example */
    (void)self;

    uint8_t *ptr = (uint8_t *)data;
    uint8_t i;
    //NSS = 0;
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET );
    {
        /* SX1272/6 clear the MSb of address to indicate read */

    	SPI_InOut(addr&0x7fU);

  //	 LDL_INFO("SPI Receive %d",addr)

  	  for(i=0U; i < size; i++){

      ptr[i]=SPI_InOut(0);

 // 	 LDL_INFO("SPI Receive %d",ptr[i])
        }
    }
    //NSS = 1;
    HAL_GPIO_WritePin( NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET );
}


//UNCOM chip reset
void LDL_Chip_reset(void *self, bool state)
{

 __HAL_RCC_GPIOA_CLK_ENABLE();


   GPIO_InitTypeDef initStruct = { 0 };

   initStruct.Pin = Reset_Pin ;
   initStruct.Mode =GPIO_MODE_OUTPUT_PP;
   initStruct.Pull = GPIO_NOPULL;
   initStruct.Speed = GPIO_SPEED_HIGH;
   HAL_GPIO_Init(Reset_GPIO_Port, &initStruct);
     if(state){

         // pull down
           HAL_GPIO_Init(Reset_GPIO_Port, &initStruct);

           HAL_GPIO_WritePin( Reset_GPIO_Port, Reset_Pin, SET );
     }
     else{
           initStruct.Mode = GPIO_NOPULL;
           HAL_GPIO_Init(Reset_GPIO_Port, &initStruct);
         // hiz
     }
}
////////////////////////

//UNCOM EXTI_CALLBACkS
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
	case DIO0_Pin:
		LDL_Radio_interrupt(&radio, 0);
		break;
	case DIO1_Pin:
		LDL_Radio_interrupt(&radio, 1);
		break;
	case DIO2_Pin:
		LDL_Radio_interrupt(&radio, 2);
		break;
	case DIO3_Pin:
		LDL_Radio_interrupt(&radio, 3);
		break;
	default:
		break;
	}
}

void enable_interrupts(void){

	  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

	  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	  HAL_NVIC_SetPriority(LPTIM1_IRQn, 0, 0);
	  HAL_NVIC_EnableIRQ(LPTIM1_IRQn);

}
//////////////////// application handler


void app_handler(void *app, enum ldl_mac_response_type type, const union ldl_mac_response_arg *arg)
{

    switch(type){

    /* Some random is required for dithering channels and scheduling
     *
     * Applications that have no better source of entropy will use this
     * event to seed the stdlib random generator.
     *
     * */
    case LDL_MAC_STARTUP:
    	LDL_INFO(TAG,"LDL_MAC_STARTUP");
        srand(arg->startup.entropy);
        break;

    /* this is data from confirmed/unconfirmed down frames */
    case LDL_MAC_RX:
    	LDL_INFO(TAG,"LDL_MAC_RX");
        (void)arg->rx.port;
        (void)arg->rx.data;
        (void)arg->rx.size;
        break;

    /* an opportunity for application to cache session */
    case LDL_MAC_SESSION_UPDATED:
    	LDL_INFO(TAG,"LDL_MAC_SESSION_UPDATED");
        (void)arg->session_updated.session;
        break;

    /* an opportunity for the application to:
     *
     * - cache the joinNonce
     * - cache the next devNonce
     * - cache session keys
     * - view join parameters (which are stored as part of session state)
     *
     * */
    case LDL_MAC_JOIN_COMPLETE:
    	LDL_INFO(TAG, "LDL_MAC_JOIN_COMPLETE nextDevNonce=%d joinNonce=%d", arg->join_complete.nextDevNonce, arg->join_complete.joinNonce);


        break;

    case LDL_MAC_CHIP_ERROR:LDL_INFO(TAG,"LDL_MAC_CHIP_ERROR");
    case LDL_MAC_RESET:LDL_INFO(TAG,"LDL_MAC_RESET");
    case LDL_MAC_JOIN_TIMEOUT:LDL_INFO(TAG,"LDL_MAC_JOIN_TIMEOUT");
    case LDL_MAC_DATA_COMPLETE:LDL_INFO(TAG,"LDL_MAC_DATA_COMPLETE");
    case LDL_MAC_DATA_TIMEOUT:LDL_INFO(TAG,"LDL_MAC_DATA_TIMEOUT");
    case LDL_MAC_DATA_NAK:LDL_INFO(TAG,"LDL_MAC_DATA_NAK");
    case LDL_MAC_LINK_STATUS:LDL_INFO(TAG,"LDL_MAC_LINK_STATUS");
    case LDL_MAC_RX1_SLOT:LDL_INFO(TAG,"LDL_MAC_RX1_SLOT");
    case LDL_MAC_RX2_SLOT:LDL_INFO(TAG,"LDL_MAC_RX2_SLOT");
    case LDL_MAC_TX_COMPLETE:LDL_INFO(TAG,"LDL_MAC_TX_COMPLETE");
    case LDL_MAC_TX_BEGIN:LDL_INFO(TAG, "LDL_MAC_TX_BEGIN devNonce=%d joinNonce=%d", mac.devNonce, mac.joinNonce);
    default:
        break;
    }
}

uint32_t GetTimer(){

	uint32_t temp2 =(uint16_t)Lpcounter;
	temp2 = (temp2 << 16);
	temp2|= HAL_LPTIM_ReadCounter(&hlptim1);

	return temp2;
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  /* Prevent unused argument(s) compilation warning */
	Lpcounter++;
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_LPTIM_AutoReloadWriteCallback could be implemented in the user file
   */
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(char const * caller_name)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
