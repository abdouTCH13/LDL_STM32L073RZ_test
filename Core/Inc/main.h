/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Reset_Pin GPIO_PIN_0
#define Reset_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define MISO_Pin GPIO_PIN_6
#define MISO_GPIO_Port GPIOA
#define MOSI_Pin GPIO_PIN_7
#define MOSI_GPIO_Port GPIOA
#define DIO0_Pin GPIO_PIN_10
#define DIO0_GPIO_Port GPIOA
#define DIO0_EXTI_IRQn EXTI4_15_IRQn
#define DIO1_Pin GPIO_PIN_3
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI2_3_IRQn
#define DIO3_Pin GPIO_PIN_4
#define DIO3_GPIO_Port GPIOB
#define DIO3_EXTI_IRQn EXTI4_15_IRQn
#define DIO2_Pin GPIO_PIN_5
#define DIO2_GPIO_Port GPIOB
#define DIO2_EXTI_IRQn EXTI4_15_IRQn
#define NSS_Pin GPIO_PIN_6
#define NSS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define HTS221_I2C_ADDRESS  (uint8_t)0xBE


#define HTS221_ADDRESS_READ  (uint8_t)0xBF
#define HTS221_ADDRESS_WRITE  (uint8_t)0xBE


////////////
#define HTS221_WHO_AM_I_REG          (uint8_t)0x0F

#define HTS221_WHO_AM_I_VAL         (uint8_t)0xBC

//////////



#define HTS221_AV_CONF_REG        (uint8_t)0x10

#define HTS221_AVGT           0x38
#define HTS221_AVGH           0x7


////////////////////////


#define HTS221_CTRL_REG1      (uint8_t)0x20

#define HTS221_PD        0x80
#define HTS221_BDU       0x0
#define HTS221_ODR       0x1


////////////////////////////



#define HTS221_CTRL_REG2      (uint8_t)0x21



//////////////////////////////////



#define HTS221_CTRL_REG3      (uint8_t)0x22



//////////////////////////


#define HTS221_STATUS_REG    (uint8_t)0x27



//////////////////////


#define HTS221_HR_OUT_L_REG        (uint8_t)0x28

#define HTS221_HR_OUT_H_REG        (uint8_t)0x29


////////////////////////


#define HTS221_TEMP_OUT_L_REG         (uint8_t)0x2A

#define HTS221_TEMP_OUT_H_REG         (uint8_t)0x2B

//////////////////////////

#define HTS221_H0_RH_X2        (uint8_t)0x30
#define HTS221_H1_RH_X2        (uint8_t)0x31
#define HTS221_T0_DEGC_X8      (uint8_t)0x32
#define HTS221_T1_DEGC_X8      (uint8_t)0x33
#define HTS221_T0_T1_DEGC_H2   (uint8_t)0x35
#define HTS221_H0_T0_OUT_L     (uint8_t)0x36
#define HTS221_H0_T0_OUT_H     (uint8_t)0x37
#define HTS221_H1_T0_OUT_L     (uint8_t)0x3A
#define HTS221_H1_T0_OUT_H     (uint8_t)0x3B
#define HTS221_T0_OUT_L        (uint8_t)0x3C
#define HTS221_T0_OUT_H        (uint8_t)0x3D
#define HTS221_T1_OUT_L        (uint8_t)0x3E
#define HTS221_T1_OUT_H        (uint8_t)0x3F

///////////////////////









/*---------------------------------Addresses  LPS25HB Pressure sensor----------------------------------------------------------*/
#define LPS25HB_ADDRESS 							0x5C	//Note that SA0 = 0 so LPS25HB_ADDRESS=SAD is 1011100= (0x5C)h
                                            //SAD+R/W=?==>>SAD+R=(B9h) and SAD+w=(B8h)
#define LPS25HB_ADDRESS_READ		0xB9
#define LPS25HB_ADDRESS_WRITE		0xB8
#define LPS25HB_DEVICE_ID							0xBD	//Device ID, the Default value in the WHO_AM_I 	Register =	0xBD



//************************************************************************************************************************//
/*---------------------------------Register Locations LPS25HB Pressure sensor--------------------------------------------------*/
#define LPS25HB_WHO_AM_I				  0x0F	//Who am I register location
#define LPS25HB_RES_CONF				  0x10	//Pressure and temperature Resolution
#define LPS25HB_CTRL_REG1				  0x20	//Contains PD, BDU and more
#define LPS25HB_CTRL_REG2				  0x21	//Contains one-shot mode and FIFO settings
#define LPS25HB_CTRL_REG3				  0x22	//Contains  Interrupt configurations
#define LPS25HB_CTRL_REG4         0x23  // REG4 Interrupt configuration
#define LPS25HB_INTERRUPT_CFG			0x24	//interrupt configuration register
#define LPS25HB_INTERRUPT_Source	0x25  // Interrupt source
#define LPS25HB_STATUS_REG				0x27	//Tells whether the Pressure Data is ready or is being overrun
#define LPS25HB_PRESS_OUT_XL			0x28	//(LSB) Pressure output value
#define LPS25HB_PRESS_OUT_L				0x29    //(mid part) Pressure output value
#define LPS25HB_PRESS_OUT_H				0x2A	//(MSB) Pressure output value
#define LPS25HB_THS_P_L					  0x30	//contains threshold least significant bit
#define LPS25HB_THS_P_H					  0x31






#define LPS25HB_PRESS_REF_XL			0x08	//(LSB) Pressure output value
#define LPS25HB_PRESS_REF_L				0x09    //(mid part) Pressure output value
#define LPS25HB_PRESS_REF_H				0x0A	//(MSB) Pressure output value


/*---------------------------------Configuration Bits LPS25HB Pressure sensor-------------------------------------------------*/
#define LPS25HB_CTRL_REG1_PD			0x80	//Power Down when 0, active mode when 1 (Default 0); PD=1==>>1 active mode is selected
#define LPS25HB_CTRL_REG1_BDU			0x4		//Block Data Update: 0 Continuous mode, 1 read LSB,Mid,MSB first; BDU=1 ==>> output registers not updated until MSB and LSB have been read)

#define LPS25HB_CTRL_REG1_ODR_ONE_SHOT_MODE_ENABLED 0x00
#define LPS25HB_CTRL_REG1_ODR_1HZ		                0x10
#define LPS25HB_CTRL_REG1_ODR_7HZ		                0x20
#define LPS25HB_CTRL_REG1_ODR_12HZ		              0x30
#define LPS25HB_CTRL_REG1_ODR_25HZ		              0x40
#define LPS25HB_CTRL_REG1_RESET_AZ		              0x2
#define LPS25HB_CTRL_REG1_DIF_E			                0x8


#define LPS25HB_CTRL_REG2_ONE_SHOT		              0x1		//One shot mode enabled, obtains a new dataset
#define LPS25HB_CTRL_REG2_AUTOZERO		              0x2

#define LPS25HB_CTRL_REG3_INT_H_L                   0x00 // 0: Interrupt active high 1: Interrupt active low
#define LPS25HB_CTRL_REG3_INT_S_Pressure_LH		      0x3  // INT_S[2:1]=11: Pressure low OR high (Data signal on INT_DRDY pin control bits.)

#define LPS25HB_CTRL_REG4_DRDY                      0x0 // Data-ready signal on INT_DRDY pin. Default value: 0.(0: disable; 1: enable)



#define LPS25HB_INTERRUPT_CFG_LIR		     0x4  // ???????????????  0 Demande d'interruption de verrouillage au registre INT_SOURCE (25h).
 // Valeur par défaut: 0.(0: demande d'interruption non verrouillée;1: demande d'interruption verrouillée)
#define LPS25HB_INTERRUPT_CFG_PL_E		   0x2
#define LPS25HB_INTERRUPT_CFG_PH_E		   0x1


#define LPS25HB_RES_CONF_AVGP0			0x1		//Pressure resolution Configuration
#define LPS25HB_RES_CONF_AVGP1			0x2		//Pressure resolution Configuration. AVGP0=AVGP1=1==>> Nr. internal average=512
#define LPS25HB_STATUS_REG_PDA			0x2		//Pressure data available;(0: new data for pressure is not yet available;1: new data for pressure is available)
                                            // PDA=1 is selected





#define VECTORTABLE_ALIGNMENT   (0x100U)

#define THRESHOLD ((uint16_t)6)



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
