/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define KEY_L1_Pin GPIO_PIN_0
#define KEY_L1_GPIO_Port GPIOA
#define KEY_L2_Pin GPIO_PIN_1
#define KEY_L2_GPIO_Port GPIOA
#define KEY_L3_Pin GPIO_PIN_2
#define KEY_L3_GPIO_Port GPIOA
#define KEY_L4_Pin GPIO_PIN_3
#define KEY_L4_GPIO_Port GPIOA
#define KEY_R1_Pin GPIO_PIN_4
#define KEY_R1_GPIO_Port GPIOA
#define KEY_R2_Pin GPIO_PIN_5
#define KEY_R2_GPIO_Port GPIOA
#define KEY_R3_Pin GPIO_PIN_6
#define KEY_R3_GPIO_Port GPIOA
#define KEY_R4_Pin GPIO_PIN_7
#define KEY_R4_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_0
#define LCD_RS_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_1
#define LCD_RW_GPIO_Port GPIOB
#define LCD_EN_Pin GPIO_PIN_2
#define LCD_EN_GPIO_Port GPIOB
#define LCD_D7_Pin GPIO_PIN_10
#define LCD_D7_GPIO_Port GPIOB
#define BUS_REQ_Pin GPIO_PIN_12
#define BUS_REQ_GPIO_Port GPIOB
#define RST_OUT_Pin GPIO_PIN_13
#define RST_OUT_GPIO_Port GPIOB
#define KEY_RST_Pin GPIO_PIN_15
#define KEY_RST_GPIO_Port GPIOA
#define LCD_D0_Pin GPIO_PIN_3
#define LCD_D0_GPIO_Port GPIOB
#define LCD_D1_Pin GPIO_PIN_4
#define LCD_D1_GPIO_Port GPIOB
#define LCD_D2_Pin GPIO_PIN_5
#define LCD_D2_GPIO_Port GPIOB
#define LCD_D3_Pin GPIO_PIN_6
#define LCD_D3_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_7
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_8
#define LCD_D5_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_9
#define LCD_D6_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
