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
#define RX_CMD_SIZE 10
#define RX_CMD_BUFFER_SIZE 20
uint8_t aTxBuffer[60] = "USART DMA Example: Communication between two USART using DMA";
uint8_t aRxBuffer [RX_CMD_BUFFER_SIZE][RX_CMD_SIZE*2];
uint8_t msgRcvd[RX_CMD_BUFFER_SIZE][RX_CMD_SIZE];
uint8_t idx = 0;
uint8_t RxBuffer2[BUFFERSIZE];
uint8_t NbrOfDataToRead = 10;
__IO uint32_t TimeOut = 0x0;   

//PWM
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0,
		Channel4Pulse = 0;
float rcSpeed[4] = { 0 };
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

typedef enum {LEVELROLL = 0, LEVELPITCH = 1, LEVELGYROROLL = 2, LEVELGYROPITCH = 3} stabilisation;

#define MAX_STAB 4
#define WIND_UP_GUARD 500
#define RC_MIN 1000
#define RC_MAX 2000
#define RC_ANGLE_MIN (-45)
#define RC_ANGLE_MAX 45
#define LEVEL_LIMIT 150
typedef struct PIDdata {
	float P;
	float I;
        float D;
        float integratedError;
        float lastPosition;
} PIDdata;

PIDdata PID[MAX_STAB];

#define PITCH 0
#define ROLL 1
#define YAW 2
#define INTENS 3
int16_t rcBluetooth[4] = {1500,1500,1500,1500};
float levelRoll;
    float levelPitch;

float motor[2];
  float error;
  float dTerm;
float angl;
/* Private function prototypes -----------------------------------------------*/
void TIM1_Config();
void PWM1_Config(int period);
void PWM_SetDC(uint16_t channel, uint16_t dutycycle);
static void USART_Config(void);
static void SysTickConfig(void);
static void stopRXDMA(void);
static void startRXDMAForSize(uint32_t adress,uint32_t size);
static void sendTxDMA(uint32_t adress,uint32_t size);
void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro);
void calibrate_sensors();
unsigned long millis();
void Delay(volatile uint32_t nCount);
void zeroError();
float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters,float dt);
float constrain(float value, float borderLow, float borderHigh);
float getAngleFromRC(int16_t rcValue);
void getLastSpeedFromMsg();
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
  
  //PWM config (motor control)
  TIM1_Config();
  PWM1_Config(10000);
  
  /* Tamper Button Configuration ---------------------------------------------*/
  STM_EVAL_PBInit(BUTTON_USER,BUTTON_MODE_GPIO);
    
  //Set motor speed
  PWM_SetDC(1, SPEED_100); //PE9 | PC6//ON 2ms
  PWM_SetDC(2, SPEED_100); //PE11 | PC 7
  PWM_SetDC(3, SPEED_100); //PE13
  PWM_SetDC(4, SPEED_100); //PE14

  //  /* Wait until Tamper Button is released */
  while (STM_EVAL_PBGetState(BUTTON_USER));  
  
  PWM_SetDC(1, SPEED_0); //PE9 | PC6//ON 2ms
  PWM_SetDC(2, SPEED_0); //PE11 | PC 7
  PWM_SetDC(3, SPEED_0); //PE13
  PWM_SetDC(4, SPEED_0); //PE14

  /* Initialization of the accelerometer -------------------------------------*/
  MPU6050_I2C_Init();
  MPU6050_Initialize();

  if (MPU6050_TestConnection()) {
		// connection success
		STM_EVAL_LEDOff(LED3);
  }else{
                STM_EVAL_LEDOff(LED4);
  }

  //Calibration process
  //  Use the following global variables and access functions
  //  to calibrate the acceleration sensor
  calibrate_sensors();

  zeroError();
  
  //Ready to receive message
  /* Enable DMA USART RX Stream */
  DMA_Cmd(USARTx_RX_DMA_STREAM,ENABLE);
  /* Enable USART DMA RX Requsts */
  USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
  
  while(1){
    //--------------------------------------------------------
    //------ Used to configure the speed controller ----------
    //--------------------------------------------------------
    
    // press blue button to force motor at SPEED_100
    if (STM_EVAL_PBGetState(BUTTON_USER)){
      PWM_SetDC(1, SPEED_100); //PE9 | PC6//ON 2ms
      PWM_SetDC(2, SPEED_100); //PE11 | PC 7
      PWM_SetDC(3, SPEED_100); //PE13
      PWM_SetDC(4, SPEED_100); //PE14
      
      //  /* Wait until Tamper Button is released */
      while (STM_EVAL_PBGetState(BUTTON_USER));  
      
      PWM_SetDC(1, SPEED_0); //PE9 | PC6//ON 2ms
      PWM_SetDC(2, SPEED_0); //PE11 | PC 7
      PWM_SetDC(3, SPEED_0); //PE13
      PWM_SetDC(4, SPEED_0); //PE14
      
      Delay(100);
    }
    
    //--------------------------------------------------------
    //------ Get gyro information                   ----------
    //--------------------------------------------------------
    
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

   //Stabilisation
   // Stable Mode
    angl = getAngleFromRC(rcBluetooth[ROLL]);
    levelRoll = (getAngleFromRC(rcBluetooth[ROLL]) - angle_x) * PID[LEVELROLL].P;
    levelPitch = (getAngleFromRC(rcBluetooth[PITCH]) - angle_y) * PID[LEVELPITCH].P;
    // Check if pilot commands are not in hover, don't auto trim
//    if ((abs(receiver.getTrimData(ROLL)) > levelOff) || (abs(receiver.getTrimData(PITCH)) > levelOff)) {
//      zeroIntegralError();
//    }
//    else {
      PID[LEVELROLL].integratedError = constrain(PID[LEVELROLL].integratedError + (((getAngleFromRC(rcBluetooth[ROLL]) - angle_x) * dt) * PID[LEVELROLL].I), -LEVEL_LIMIT, LEVEL_LIMIT);
      PID[LEVELPITCH].integratedError = constrain(PID[LEVELPITCH].integratedError + (((getAngleFromRC(rcBluetooth[PITCH]) + angle_y) * dt) * PID[LEVELROLL].I), -LEVEL_LIMIT, LEVEL_LIMIT);
//    }
    //motors.setMotorAxisCommand(ROLL,
    motor[ROLL] = updatePID(rcBluetooth[ROLL] + levelRoll, gyro_x + 1500, &PID[LEVELGYROROLL],dt) + PID[LEVELROLL].integratedError;//);
    //motors.setMotorAxisCommand(PITCH,
    motor[PITCH] = updatePID(rcBluetooth[PITCH] + levelPitch, gyro_y + 1500, &PID[LEVELGYROPITCH],dt) + PID[LEVELPITCH].integratedError;//);
   
    getLastSpeedFromMsg(); 
    
    PWM_SetDC(1, SPEED_0 + SPEED_RANGE*rcSpeed[1] + motor[ROLL] *0.10f); //PE9 | PC6//ON 2ms
  
        //Send data on UART
    *(float*)(aTxBuffer) = angle_x;
    *(float*)(aTxBuffer+4) = angle_y;
    *(float*)(aTxBuffer+8) = angle_z;
    *(float*)(aTxBuffer+12) = motor[ROLL];
    *(float*)(aTxBuffer+16) =  motor[PITCH];
   sendTxDMA((uint32_t)aTxBuffer,20);
   
   //Wait a little bit
   Delay(3); //30 ms
   
  }
}

void getLastSpeedFromMsg(){
    uint16_t i = 0,a = idx,cnt = 3;
    do{
    for(i=0;i<RX_CMD_SIZE-4;i++){
      if(msgRcvd[a][i]=='U'){
        rcSpeed[0] = msgRcvd[a][i+1];
        rcSpeed[1] = msgRcvd[a][i+2];
        rcSpeed[2] = msgRcvd[a][i+3];
        rcSpeed[3] = msgRcvd[a][i+4];
        return;
      }
    }
    a = (a-1+RX_CMD_BUFFER_SIZE)%RX_CMD_BUFFER_SIZE;
    }while(cnt--);
      rcSpeed[0] = 0;
        rcSpeed[1] = 0;
        rcSpeed[2] = 0;
        rcSpeed[3] = 0;
}

float val;

float getAngleFromRC(int16_t rcValue){
    val = (int16_t)constrain(rcValue,RC_MIN,RC_MAX);
    val = RC_ANGLE_MIN+(val-RC_MIN)/(RC_MAX-RC_MIN)*(RC_ANGLE_MAX-RC_ANGLE_MIN);
    return val;
}

float updatePID(float targetPosition, float currentPosition, struct PIDdata *PIDparameters,float dt) {

  error = targetPosition - currentPosition;
  
  PIDparameters->integratedError += error * dt;
  PIDparameters->integratedError = constrain(PIDparameters->integratedError, -WIND_UP_GUARD, WIND_UP_GUARD);
  
  dTerm = PIDparameters->D * (currentPosition - PIDparameters->lastPosition) / dt;
  PIDparameters->lastPosition = currentPosition;
  return (PIDparameters->P * error) + (PIDparameters->I * (PIDparameters->integratedError)) + dTerm;
}

float constrain(float value, float borderLow, float borderHigh){
  if(value < borderLow)return borderLow;
  if(value > borderHigh) return borderHigh;
  return value;
}
void zeroError() {
  int8_t axis;
  for (axis = LEVELROLL; axis < MAX_STAB; axis++){
    PID[axis].P = 3;
    PID[axis].I = 0.8;
    PID[axis].D = 0.7;
    PID[axis].lastPosition = 0;
    PID[axis].integratedError = 0;
  }
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

void PWM_SetDC(uint16_t channel, uint16_t dutycycle) {
        
	if (channel == 1) {
		TIM3->CCR1 = dutycycle;
		TIM1->CCR1 = dutycycle;
	} else if (channel == 2) {
		TIM3->CCR2 = dutycycle;
		TIM1->CCR2 = dutycycle;
	} else if (channel == 3) {
		TIM3->CCR3 = dutycycle;
		TIM1->CCR3 = dutycycle;
	} else if (channel == 4) {
		TIM3->CCR4 = dutycycle;
		TIM1->CCR4 = dutycycle;
	}
}

/**
 * @brief  Configure the TIM1 Pins.
 * @param  None
 * @retval None
 */
void TIM1_Config(void) {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA and GPIOB clocks enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* GPIOA Configuration: Channel 1 to 4 as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//  /* Connect TIM pins to AF1 */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
}

void PWM1_Config(int period) {

	/* -----------------------------------------------------------------------
	 1/ Generate 3 complementary PWM signals with 3 different duty cycles:

	 In this example TIM1 input clock (TIM1CLK) is set to 2 * APB2 clock (PCLK2),
	 since APB2 prescaler is different from 1 (APB2 Prescaler = 2, see system_stm32f4xx.c file).
	 TIM1CLK = 2 * PCLK2
	 PCLK2 = HCLK / 2
	 => TIM1CLK = 2*(HCLK / 2) = HCLK = SystemCoreClock

	 To get TIM1 counter clock at 168 MHz, the prescaler is computed as follows:
	 Prescaler = (TIM1CLK / TIM1 counter clock) - 1
	 Prescaler = (SystemCoreClock / 168 MHz) - 1 = 0

	 The objective is to generate PWM signal at 17.57 KHz:
	 - TIM1_Period = (SystemCoreClock / 17570) - 1

	 To get TIM1 output clock at 17.57 KHz, the period (ARR) is computed as follows:
	 ARR = (TIM1 counter clock / TIM1 output clock) - 1
	 = 9561

	 The Three Duty cycles are computed as the following description:

	 TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = 50%
	 TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 25%
	 TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR)* 100 = 12.5%

	 The Timer pulse is calculated as follows:
	 - TIM1_CCRx = (DutyCycle * TIM1_ARR)/ 100

	 2/ Insert a dead time equal to (11/SystemCoreClock) ns

	 3/ Configure the break feature, active at High level, and using the automatic
	 output enable feature

	 4/ Use the Locking parameters level1.

	 Note:
	 SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
	 Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
	 function to update SystemCoreClock variable value. Otherwise, any configuration
	 based on this variable will be incorrect.
	 ----------------------------------------------------------------------- */

	/* Time Base configuration */
	uint16_t PrescalerValue = 0;
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t)((SystemCoreClock / 2) / 1600000) - 1;

	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* Channel 1to 4 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	/* Automatic Output enable, Break, dead time and lock configuration*/
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 11;
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);

	/* Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
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
  DMA_InitStructure.DMA_BufferSize = RX_CMD_SIZE ;
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
            for(;i<RX_CMD_SIZE;i++){
              if(aRxBuffer[idx][i]=='U'){
                memcpy(msgRcvd[idx],aRxBuffer[idx]+i,RX_CMD_SIZE);
                STM_EVAL_LEDToggle(LED6);
                break;
              }
            }
            idx=(idx+1)%RX_CMD_BUFFER_SIZE;
            startRXDMAForSize((uint32_t) aRxBuffer[idx],RX_CMD_SIZE);
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
 * Delay a number of systick cycles (10ms)
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
