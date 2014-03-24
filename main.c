/**
  ******************************************************************************
  * @file    USART/USART_TwoBoards/USART_DataExchangeDMA/main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    18-January-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc16.h"
#include "MPU6050.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
/** @addtogroup STM32F4xx_StdPeriph_Examples
* @{
*/

/** @addtogroup USART_DataExchangeDMA
* @{
*/

/* Private typedef -----------------------------------------------------------*/
static DMA_InitTypeDef  DMA_InitStructure;

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile uint32_t time_var1, time_var2;
uint8_t aTxBuffer[60] = "USART DMA Example: Communication between two USART using DMA";
uint8_t aRxBuffer [300][BUFFERSIZE];
uint8_t msgRcvd[300][60];
uint8_t idx = 0;
uint8_t RxBuffer2[BUFFERSIZE];
uint8_t NbrOfDataToRead = 10;
__IO uint32_t TimeOut = 0x0;   

int16_t AccelGyro[8] = { 0 };
double tempC;
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;
float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;
float gyro_x;
float gyro_y;
float gyro_z;
float accel_x;
float accel_y;
float accel_z;
float lastAngle[3];
float lastGyroAngle[3];
unsigned long last_read_time;
float angle_x;
float angle_y;
float angle_z;
float dt;

/* Private function prototypes -----------------------------------------------*/
static void USART_Config(void);
static void SysTickConfig(void);
static void stopRXDMA(void);
static void startRXDMAForSize(uint32_t adress,uint32_t size);
static void sendTxDMA(uint32_t adress,uint32_t size);
void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro);
void calibrate_sensors();
unsigned long millis();
void Delay(volatile uint32_t nCount);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       files (startup_stm32f40xx.s/startup_stm32f427x.s) before to branch to 
       application main. 
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */
  
  /* USART configuration -----------------------------------------------------*/
  USART_Config();
    
  /* SysTick configuration ---------------------------------------------------*/
  SysTickConfig();
  
  /* LEDs configuration ------------------------------------------------------*/
  STM_EVAL_LEDInit(LED3);
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED6);
  
  STM_EVAL_LEDOn(LED3);//orange
  STM_EVAL_LEDOn(LED4);//verte
  STM_EVAL_LEDOn(LED5);//rouge
  STM_EVAL_LEDOn(LED6);//bleue
 
  /* Tamper Button Configuration ---------------------------------------------*/
  STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_GPIO);

  /* Initialization of the accelerometer -------------------------------------*/
  MPU6050_I2C_Init();
  MPU6050_Initialize();

  if (MPU6050_TestConnection() == TRUE) {
		// connection success
		STM_EVAL_LEDOff(LED3);
  }else{
                STM_EVAL_LEDOff(LED4);
  }

  //Calibration process
  //  Use the following global variables and access functions
  //  to calibrate the acceleration sensor
  calibrate_sensors();

  while(1){
    //int error;
    // Read the raw values.
    MPU6050_GetRawAccelGyro(AccelGyro);

    // Get the time of reading for rotation computations
    unsigned long t_now = millis();
    STM_EVAL_LEDToggle(LED5);
    // The temperature sensor is -40 to +85 degrees Celsius.
    // It is a signed integer.
    // According to the datasheet:
    //   340 per degrees Celsius, -512 at 35 degrees.
    // At 0 degrees: -512 – (340 * 35) = -12412
    //dT = ( (double) AccelGyro[TEMP] + 12412.0) / 340.0;

    // Convert gyro values to degrees/sec
    gyro_x = (AccelGyro[GYRO_X] - base_x_gyro) / FSSEL;
    gyro_y = (AccelGyro[GYRO_Y] - base_y_gyro) / FSSEL;
    gyro_z = (AccelGyro[GYRO_Z] - base_z_gyro) / FSSEL;

    // Get raw acceleration values
    accel_x = AccelGyro[ACC_X];
    accel_y = AccelGyro[ACC_Y];
    accel_z = AccelGyro[ACC_Z];

    // Get angle values from accelerometer
    //float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
    float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS2DEGREES;
    float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS2DEGREES;

    //float accel_angle_z = 0;

    //// Compute the (filtered) gyro angles
    //Get the value in second, a tick is every 10ms
    dt = (t_now - last_read_time)/100.0;
    float gyro_angle_x = gyro_x*dt + lastAngle[X];//get_last_x_angle();
    float gyro_angle_y = gyro_y*dt + lastAngle[Y];//(get_last_y_angle();
    float gyro_angle_z = gyro_z*dt + lastAngle[Z];//get_last_z_angle();

    // Compute the drifting gyro angles
    float unfiltered_gyro_angle_x = gyro_x*dt + lastGyroAngle[X];//get_last_gyro_x_angle();
    float unfiltered_gyro_angle_y = gyro_y*dt + lastGyroAngle[Y];//get_last_gyro_y_angle();
    float unfiltered_gyro_angle_z = gyro_z*dt + lastGyroAngle[Z];//get_last_gyro_z_angle();

    // Apply the complementary filter to figure out the change in angle – choice of alpha is
    // estimated now.  Alpha depends on the sampling rate…
    float alpha = 0.96;
    angle_x = alpha * gyro_angle_x + (1.0 - alpha) * accel_angle_x;
    angle_y = alpha * gyro_angle_y + (1.0 - alpha) * accel_angle_y;
    angle_z = gyro_angle_z;  //Accelerometer doesn’t give z-angle

    //printf("%4.2f %4.2f %4.2f\r\n",angle_x,angle_y,angle_z);

    //// Update the saved data with the latest values
    set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

    //Send data on UART
    *(float*)(aTxBuffer) = angle_x;
    *(float*)(aTxBuffer+4) = angle_y;
    *(float*)(aTxBuffer+8) = angle_z;

   sendTxDMA((uint32_t)aTxBuffer,12);

   Delay(2); //20 ms
   
  }
  
//  /* Wait until Tamper Button is pressed */
//  while (!STM_EVAL_PBGetState(BUTTON_USER));  
//
//#ifdef USART_RECEIVER
//  
//   
//    /* Enable DMA USART RX Stream */
//  DMA_Cmd(USARTx_RX_DMA_STREAM,ENABLE);
//  
//  /* Enable USART DMA RX Requsts */
//  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
//  
//  /* Waiting the end of Data transfer */
//  while (USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);
//    
//  while(1){}
//
//#endif /* USART_RECEIVER */

}

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
last_read_time = time;
lastAngle[X] = x;
lastAngle[Y] = y;
lastAngle[Z] = z;
lastGyroAngle[X] = x_gyro;
lastGyroAngle[Y] = y_gyro;
lastGyroAngle[Z] = z_gyro;
}

// The sensor should be motionless on a horizontal surface
//  while calibration is happening
void calibrate_sensors() {
int num_readings = 10;
float x_accel = 0;
float y_accel = 0;
float z_accel = 0;
float x_gyro = 0;
float y_gyro = 0;
float z_gyro = 0;

// Discard the first set of values read from the IMU
MPU6050_GetRawAccelGyro(AccelGyro);

// Read and average the raw values from the IMU
for (int i = 0; i < num_readings; i++) {
  MPU6050_GetRawAccelGyro(AccelGyro);
  x_accel += AccelGyro[ACC_X];
  y_accel += AccelGyro[ACC_Y];
  z_accel += AccelGyro[ACC_Z];
  x_gyro += AccelGyro[GYRO_X];
  y_gyro += AccelGyro[GYRO_Y];
  z_gyro += AccelGyro[GYRO_Z];
  Delay(10);
}
x_accel /= num_readings;
y_accel /= num_readings;
z_accel /= num_readings;
x_gyro /= num_readings;
y_gyro /= num_readings;
z_gyro /= num_readings;

// Store the raw calibration values globally
base_x_accel = x_accel;
base_y_accel = y_accel;
base_z_accel = z_accel;
base_x_gyro = x_gyro;
base_y_gyro = y_gyro;
base_z_gyro = z_gyro;

//Serial.println("Finishing Calibration");
}

/**
  * @brief  Configures the USART Peripheral.
  * @param  None
  * @retval None
  */
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USARTx_CLK_INIT(USARTx_CLK, ENABLE);
  
  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(USARTx_DMAx_CLK, ENABLE);
  
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
  GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
  GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
  GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);
 
  /* USARTx configuration ----------------------------------------------------*/
  /* Enable the USART OverSampling by 8 */
  USART_OverSampling8Cmd(USARTx, ENABLE); 
  
  /* USARTx configured as follow:
        - BaudRate = 5250000 baud
		   - Maximum BaudRate that can be achieved when using the Oversampling by 8
		     is: (USART APB Clock / 8) 
			 Example: 
			    - (USART3 APB1 Clock / 8) = (42 MHz / 8) = 5250000 baud
			    - (USART1 APB2 Clock / 8) = (84 MHz / 8) = 10500000 baud
		   - Maximum BaudRate that can be achieved when using the Oversampling by 16
		     is: (USART APB Clock / 16) 
			 Example: (USART3 APB1 Clock / 16) = (42 MHz / 16) = 2625000 baud
			 Example: (USART1 APB2 Clock / 16) = (84 MHz / 16) = 5250000 baud
        - Word Length = 8 Bits
        - one Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */ 
  USART_InitStructure.USART_BaudRate = USARTx_Baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTx, &USART_InitStructure);

  /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
   
  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_BufferSize = 10 ;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USARTx->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure TX DMA */
  DMA_InitStructure.DMA_Channel = USARTx_TX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aTxBuffer ;
  DMA_Init(USARTx_TX_DMA_STREAM,&DMA_InitStructure);
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = USARTx_RX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)aRxBuffer[idx] ; 
  DMA_Init(USARTx_RX_DMA_STREAM,&DMA_InitStructure);
  
  /*DMA NVIC*/
  DMA_ITConfig(USARTx_RX_DMA_STREAM,DMA_IT_TC,ENABLE);
  
  NVIC_InitStructure.NVIC_IRQChannel = USARTx_DMA_RX_IRQn;		 // we want to configure the DMA RX interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the DMA RX interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);						
  
  /* Enable USART */
  USART_Cmd(USARTx, ENABLE);
}

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USARTx_DMA_RX_IRQHandler(void){

	// check if the USART1 receive interrupt flag was set
	if( DMA_GetITStatus(USARTx_RX_DMA_STREAM,USARTx_RX_DMA_FLAG_TCIF)==RESET ){
              //while (DMA_GetFlagStatus(USARTx_RX_DMA_STREAM,USARTx_RX_DMA_FLAG_TCIF)==RESET);
          
          /* Clear DMA Transfer Complete Flags */
          DMA_ClearFlag(USARTx_RX_DMA_STREAM,USARTx_RX_DMA_FLAG_TCIF);
          stopRXDMA();
          uint8_t i = 0;
          for(;i<60;i++){
            if(aRxBuffer[idx][i]=='U'){
              memcpy(msgRcvd[idx],aRxBuffer[idx]+i,60);
              STM_EVAL_LEDToggle(LED6);
              break;
            }
          }
          idx++;
          startRXDMAForSize((uint32_t) aRxBuffer[idx],60);
          
	}
}

void sendTxDMA(uint32_t adress,uint32_t size){
  DMA_InitStructure.DMA_Channel = USARTx_TX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral ;
  DMA_InitStructure.DMA_Memory0BaseAddr = adress;
  DMA_InitStructure.DMA_BufferSize = size;
  DMA_Init(USARTx_TX_DMA_STREAM,&DMA_InitStructure);
  
    /* Enable DMA USART TX Stream */
  DMA_Cmd(USARTx_TX_DMA_STREAM,ENABLE);
  
  /* Enable USART DMA TX Requests */
  USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);

  /* Waiting the end of Data transfer */
  while (USART_GetFlagStatus(USARTx,USART_FLAG_TC)==RESET);    
  while (DMA_GetFlagStatus(USARTx_TX_DMA_STREAM,USARTx_TX_DMA_FLAG_TCIF)==RESET);
  
  /* Clear DMA Transfer Complete Flags */
  DMA_ClearFlag(USARTx_TX_DMA_STREAM,USARTx_TX_DMA_FLAG_TCIF);
  DMA_Cmd(USARTx_TX_DMA_STREAM, DISABLE);
  /* Clear USART Transfer Complete Flags */
  USART_ClearFlag(USARTx,USART_FLAG_TC);
  USART_DMACmd(USARTx, USART_DMAReq_Tx, DISABLE);
}

void stopRXDMA(){
  DMA_Cmd(USARTx_RX_DMA_STREAM, DISABLE);
  /* Clear USART Transfer Complete Flags */
  USART_ClearFlag(USARTx,USART_FLAG_TC);
  /* Enable USART DMA RX Requsts */
  USART_DMACmd(USARTx, USART_DMAReq_Rx, DISABLE);
}

void startRXDMAForSize(uint32_t adress,uint32_t size){
  DMA_InitStructure.DMA_Channel = USARTx_RX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr = adress;
  DMA_InitStructure.DMA_BufferSize = size;
  DMA_Init(USARTx_RX_DMA_STREAM,&DMA_InitStructure);
  /* Enable DMA USART RX Stream */
  DMA_Cmd(USARTx_RX_DMA_STREAM,ENABLE);          
  /* Enable USART DMA RX Requsts */
  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
}
  

/**
  * @brief  Configures the SysTick Base time to 10 ms.
  * @param  None
  * @retval None
  */
static void SysTickConfig(void)
{
  /* Set SysTick Timer for 10ms interrupts  */
  if (SysTick_Config(SystemCoreClock / 100))
  {
    /* Capture error */
    while (1);
  }
  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

unsigned long millis(){
  return time_var2;
}

/*
 * Called from systick handler
 */
void timing_handler() {
	if (time_var1) {
		time_var1--;
	}
	time_var2++;
}

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nCount) {
	time_var1 = nCount;
	while (time_var1) {
	};
}
/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
