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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Motor.h"
#include "Aktuator.h"
#include "PID_Driver.h"
#include "BNO08X.h"
#include "communication_full.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SEND_DATA_INTERVAL	100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
//---------------- MOTOR DECLARATION ---------------------//
extern motor_t motor_A, motor_B, motor_C, motor_D;

//---------------- ENCODER DECLARATION ---------------------//
extern encoder_t encoder_A, encoder_B, encoder_C, encoder_D;

//---------------- AKTUATOR DECLARATION --------------------//
aktuator_t aktuator;

//---------------- KINEMATIC DECLARATION ---------------------//
kinematic_t kinematic;

//---------------- PID DECLARATION ---------------------//
PIDController pid_vy;
PIDController pid_vx;
PIDController pid_vt;
PIDController pid_yaw;

//---------------- BNO08X DECLARATION ---------------------//
BNO08X_Typedef BNO08X_sensor;

// Typedef Communication PC
com_ctrl_get_t message_from_sensor;

// Step Message
uint16_t currentStep = 0;

// Offset Yaw
int16_t Yaw_Init = 0;
bool is_yaw_calibrated = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

bool run_to_point(double sx, double sy, double st, double error);
bool handle_heading(int16_t heading, int16_t error);
bool run_to_point_orientation(double sx, double sy, uint16_t heading, double error);
bool set_heading(int16_t heading, int16_t error);
bool run_to_point_with_yaw(int16_t sx, int16_t sy, uint16_t heading, int16_t error);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance  == TIM1){
		encoder_A.counter  	= __HAL_TIM_GET_COUNTER(htim);
		encoder_A.counts 	= (int16_t)encoder_A.counter;
		encoder_A.position	= encoder_A.counts/4;
	}
	if(htim->Instance  == TIM2){
		encoder_B.counter  	= __HAL_TIM_GET_COUNTER(htim);
		encoder_B.counts 	= (int16_t)encoder_B.counter;
		encoder_B.position	= encoder_B.counts/4;
	}
	if(htim->Instance  == TIM4){
		encoder_C.counter  	= __HAL_TIM_GET_COUNTER(htim);
		encoder_C.counts 	= (int16_t)encoder_C.counter;
		encoder_C.position	= encoder_C.counts/4;
	}
	if(htim->Instance  == TIM5){
		encoder_D.counter  	= __HAL_TIM_GET_COUNTER(htim);
		encoder_D.counts 	= (int16_t)encoder_D.counter;
		encoder_D.position	= encoder_D.counts/4;
	}

	// Encoder Eksternal
	kinematic.S1 = -encoder_A.position*PULSE_TO_DIST;
	kinematic.S2 = -encoder_B.position*PULSE_TO_DIST;
	kinematic.S3 = encoder_C.position*PULSE_TO_DIST;
	kinematic.S4 = -encoder_D.position*PULSE_TO_DIST;
	kinematic.V1 = -encoder_A.speed*PULSE_TO_DIST;
	kinematic.V2 = -encoder_B.speed*PULSE_TO_DIST;
	kinematic.V3 = encoder_C.speed*PULSE_TO_DIST;
	kinematic.V4 = -encoder_D.speed*PULSE_TO_DIST;

	// External Encoder (Nomor 3->x dan 4->y)
	kinematic.St = kinematic.St + agv_kinematic_ext_St(kinematic.Sx,kinematic.Sy,encoder_C.position*PULSE_TO_DIST,-encoder_D.position*PULSE_TO_DIST, 0);
	kinematic.Vt = kinematic.Vt + agv_kinematic_ext_St(kinematic.Vx,kinematic.Vy,encoder_C.speed*PULSE_TO_DIST,-encoder_D.speed*PULSE_TO_DIST, 0);
	kinematic.Sx = agv_kinematic_ext_Sx(-encoder_A.position*PULSE_TO_DIST,-encoder_B.position*PULSE_TO_DIST,encoder_C.position*PULSE_TO_DIST,-encoder_D.position*PULSE_TO_DIST, 0);
	kinematic.Sy = agv_kinematic_ext_Sy(-encoder_A.position*PULSE_TO_DIST,-encoder_B.position*PULSE_TO_DIST,encoder_C.position*PULSE_TO_DIST,-encoder_D.position*PULSE_TO_DIST, 0);
	kinematic.Vx = agv_kinematic_ext_Sx(-encoder_A.speed*PULSE_TO_DIST,-encoder_B.speed*PULSE_TO_DIST,encoder_C.speed*PULSE_TO_DIST,-encoder_D.speed*PULSE_TO_DIST, 0);
	kinematic.Vy = agv_kinematic_ext_Sy(-encoder_A.speed*PULSE_TO_DIST,-encoder_B.speed*PULSE_TO_DIST,encoder_C.speed*PULSE_TO_DIST,-encoder_D.speed*PULSE_TO_DIST, 0);
}

uint32_t CurrentTick = 0;
uint32_t SendDataTick = 0;
int astarlength = 0;
int astarid = 0;
int lastlength = 0;
int lastid = 0;
int lastcmd = 0;
int last_i = 0;
uint32_t msgid = 0;
uint32_t current_msgid = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart6){
		rx_ctrl_get(&message_from_sensor);
		astarlength = message_from_sensor.astar_length;
		astarid = message_from_sensor.astar_id;
		msgid++;
	}
}

//float kp_all = 13;
//float ki_all = 13;
//float kd_all = 1.34;
int16_t data_sx=0;
int16_t data_sy=0;
int16_t data_st=0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint8_t flag_interrupt = 0;
uint8_t last_interrupted = 0;
uint8_t x_arr[100];
uint8_t y_arr[100];
//int last_length;
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  //+++++++++++++++++++++++++++++++++ MOTOR INITIALIZATION +++++++++++++++++++++++++++++//
  // Configuration 'Motor A'
  motor_A.tim_R = &htim9;
  motor_A.tim_L = &htim9;
  motor_A.tim_number_R = TIM9;
  motor_A.tim_number_L = TIM9;
  motor_A.channel_R = 1;
  motor_A.channel_L = 2;
  motor_A.EN_PORT_R = ENR_C_GPIO_Port;
  motor_A.EN_PORT_L = ENL_C_GPIO_Port;
  motor_A.EN_PIN_R = ENR_C_Pin;
  motor_A.EN_PIN_L = ENL_C_Pin;

  // Configuration 'Motor B'
  motor_B.tim_R = &htim3;
  motor_B.tim_L = &htim3;
  motor_B.tim_number_R = TIM3;
  motor_B.tim_number_L = TIM3;
  motor_B.channel_R = 1;
  motor_B.channel_L = 2;
  motor_B.EN_PORT_R = ENR_B_GPIO_Port;
  motor_B.EN_PORT_L = ENL_B_GPIO_Port;
  motor_B.EN_PIN_R = ENR_B_Pin;
  motor_B.EN_PIN_L = ENL_B_Pin;

  // Configuration 'Motor C'
  motor_C.tim_R = &htim3;
  motor_C.tim_L = &htim3;
  motor_C.tim_number_R = TIM3;
  motor_C.tim_number_L = TIM3;
  motor_C.channel_R = 3;
  motor_C.channel_L = 4;
  motor_C.EN_PORT_R = ENR_A_GPIO_Port;
  motor_C.EN_PORT_L = ENL_A_GPIO_Port;
  motor_C.EN_PIN_R = ENR_A_Pin;
  motor_C.EN_PIN_L = ENL_A_Pin;

  // Configuration 'Motor D'
  motor_D.tim_R = &htim10;
  motor_D.tim_L = &htim11;
  motor_D.tim_number_R = TIM10;
  motor_D.tim_number_L = TIM11;
  motor_D.channel_R = 1;
  motor_D.channel_L = 1;
  motor_D.EN_PORT_R = ENR_D_GPIO_Port;
  motor_D.EN_PORT_L = ENL_D_GPIO_Port;
  motor_D.EN_PIN_R = ENR_D_Pin;
  motor_D.EN_PIN_L = ENL_D_Pin;

  //+++++++++++++++++++++++++++++++++ ENCODER INITIALIZATION ++++++++++++++++++++++++++++++//
  agv_encoder_start(encoder_A, &htim1, TIM1);
  agv_encoder_start(encoder_B, &htim2, TIM2);
  agv_encoder_start(encoder_C, &htim4, TIM4);
  agv_encoder_start(encoder_D, &htim5, TIM5);

  //+++++++++++++++++++++++++++++++++ ENCODER TO MOTOR ++++++++++++++++++++++++++++++++++++//
  motor_A.ENC = encoder_A;
  motor_B.ENC = encoder_B;
  motor_C.ENC = encoder_C;
  motor_D.ENC = encoder_D;

  //+++++++++++++++++++++++++++++++++ AKTUATOR INTIALIZATION ++++++++++++++++++++++++++++++//
  aktuator.PORT_IN1 = GPIOB;
  aktuator.PIN_IN1 = GPIO_PIN_5;
  aktuator.PORT_IN2 = GPIOC;
  aktuator.PIN_IN2 = GPIO_PIN_13;
  aktuator.PORT_IN3 = GPIOC;
  aktuator.PIN_IN3 = GPIO_PIN_14;
  aktuator.PORT_IN4 = GPIOC;
  aktuator.PIN_IN4 = GPIO_PIN_15 ;

  //+++++++++++++++++++++++++++++++++ COM START +++++++++++++++++++++++++++++++++++++++++++//
  komunikasi_ctrl_init(&huart6);
  rx_ctrl_start_get();

  //+++++++++++++++++++++++++++++++++ PID INITIALIZATION ++++++++++++++++++++++++++++++//
  // Y Axis
  pid_vy.Kp = 19;				pid_vy.Ki = 1;				pid_vy.Kd = 0.001;
  pid_vy.limMax = 600; 		pid_vy.limMin = -600; 		pid_vy.limMaxInt = 1; 	pid_vy.limMinInt = -1;
  pid_vy.T_sample = 0.1;
  PIDController_Init(&pid_vy);

  // X Axis
  pid_vx.Kp = 19;				pid_vx.Ki = 1;				pid_vx.Kd = 0.001;
  pid_vx.limMax = 600; 		pid_vx.limMin = -600; 		pid_vx.limMaxInt = 1; 	pid_vx.limMinInt = -1;
  pid_vx.T_sample = 0.1;
  PIDController_Init(&pid_vx);

  // T Axis
  pid_vt.Kp = 19;				pid_vt.Ki = 1;				pid_vt.Kd = 0.001;
  pid_vt.limMax = 600; 		pid_vt.limMin = -600; 		pid_vt.limMaxInt = 1; 	pid_vt.limMinInt = -1;
  pid_vt.T_sample = 0.1;
  PIDController_Init(&pid_vt);
    // Yaw Direction
//    pid_yaw.Kp = 40;			pid_yaw.Ki = 48;				pid_yaw.Kd = 0.02;
    pid_yaw.Kp = 15;			pid_yaw.Ki = 15;			pid_yaw.Kd = 0.45;
    pid_yaw.limMax = 800; 		pid_yaw.limMin = -800; 		pid_yaw.limMaxInt = 0.1; 	pid_yaw.limMinInt = -0.1;
    pid_yaw.T_sample = 0.1;
    PIDController_Init(&pid_yaw);


	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	//------------------------------ ONE STEP TEST --------------------------------------------------------//
	aktuator_down(aktuator);
	HAL_Delay(3000);
	agv_reset_all(motor_A, motor_B, motor_C, motor_D);
	agv_stop_all(motor_A, motor_B, motor_C, motor_D);
	aktuator_reset(aktuator);
	int len_msg = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while(1)
	  {
		//--------------------- SEND DATA TESTING ----------------------------//
	//		tx_ctrl_send_data(kinematic.V1, kinematic.V2, kinematic.V3, kinematic.V4, kinematic.S1,kinematic.S3,kinematic.S4);
		//--------------------- YAW CALIBRATION PROGRAM ----------------------------//
	//		if(!is_yaw_calibrated && message_from_sensor.yaw != 0){
	//			Yaw_Init = message_from_sensor.yaw;
	//		}

		//--------------------- SENDING DATA ODOMETRY ----------------------------//

		CurrentTick = HAL_GetTick();

		if(CurrentTick-SendDataTick > SEND_DATA_INTERVAL){
			tx_ctrl_send_Odometry(kinematic.Sx,kinematic.Sy,kinematic.St,kinematic.Vx,kinematic.Vy,kinematic.Vt);
			SendDataTick = CurrentTick;
		}

	//------------------------- TEST BENCH ----------------------------------------//
	//	  agv_reset_all(motor_A, motor_B, motor_C, motor_D);
	//	  agv_stop_all(motor_A, motor_B, motor_C, motor_D);
	//	  agv_speed_to_pwm(motor_A, 2000);
	//	  HAL_Delay(2000);
	//	  agv_reset_all(motor_A, motor_B, motor_C, motor_D);
	//	  agv_stop_all(motor_A, motor_B, motor_C, motor_D);
	//	  agv_speed_to_pwm(motor_B, 2000);
	//	  HAL_Delay(2000);
	//	  agv_reset_all(motor_A, motor_B, motor_C, motor_D);
	//	  agv_stop_all(motor_A, motor_B, motor_C, motor_D);
	//	  agv_speed_to_pwm(motor_C, 2000);
	//	  HAL_Delay(2000);
	//	  agv_reset_all(motor_A, motor_B, motor_C, motor_D);
	//	  agv_stop_all(motor_A, motor_B, motor_C, motor_D);
	//	  agv_speed_to_pwm(motor_D, 2000);
	//	  HAL_Delay(2000);

	//----------------------------- TEST KINEMATIC --------------------------------------------//

	//	  agv_inverse_kinematic(500, 0, 0, 0, motor_A, motor_B, motor_C, motor_D);
	//	  HAL_Delay(2000);
	//	  agv_inverse_kinematic(-500, 0, 0, 0, motor_A, motor_B, motor_C, motor_D);
	//	  HAL_Delay(2000);
	//	  agv_inverse_kinematic(0, 500, 0, 0, motor_A, motor_B, motor_C, motor_D);
	//	  HAL_Delay(2000);
	//	  agv_inverse_kinematic(0, -500, 0, 0, motor_A, motor_B, motor_C, motor_D);
	//	  HAL_Delay(2000);
	//	  agv_inverse_kinematic(0, 0, 500, 0, motor_A, motor_B, motor_C, motor_D);
	//	  HAL_Delay(2000);
	//	  agv_inverse_kinematic(0, 0, -500, 0, motor_A, motor_B, motor_C, motor_D);
	//	  HAL_Delay(2000);

	//----------------------------- TEST POINT --------------------------------------------//
	//	  run_to_point(0,300,0,0.1);
	//	  run_to_point_orientation(0,300,0,3);
	//	  run_to_point_with_yaw(0,5000,0,1);
	//    handle_heading(45,5);
	//	  handle_heading(180,5);
	//	  HAL_Delay(100);
	//	set_heading(90,0.1);

	//----------------------------- TEST HEADING --------------------------------------------//
	//		run_to_point_orientation(0,100,90,2);
	//----------------------------- RUNNING COMMAND --------------------------------------------//
	//	  if(currentStep <= message_from_sensor.step){
	//		  if(run_to_point(message_from_sensor.x_pos*100,message_from_sensor.y_pos*100,message_from_sensor.orientation*100,5)){
	//
	//			// Sending to PC that Task is Done
	//			currentStep = message_from_sensor.step;
	//
	//			// Sending to PC that Task is Done
	//			tx_ctrl_task_done(currentStep);
	//		  }
	//	  }
	//----------------------------- RUNNING ASTAR --------------------------------------------//
		if(lastid != message_from_sensor.astar_msg_id && (message_from_sensor.astar_id+1) == message_from_sensor.astar_length){


			len_msg = message_from_sensor.astar_total_length-1;

			for(int i = len_msg; i >= 0; i--){
				last_i = i;
//				if(flag_interrupt == 1){
//					flag_interrupt = 0;
//					i=-1;
//				}
				if(true){
					while(!run_to_point_with_yaw(message_from_sensor.astar_coordinate_x[i]*500,message_from_sensor.astar_coordinate_y[i]*500,0,10)){

						// Interrupt message from Command
						if(message_from_sensor.id_data != lastcmd){
							agv_reset_all(motor_A, motor_B, motor_C, motor_D);
							agv_stop_all(motor_A, motor_B, motor_C, motor_D);
							break;
						}
//						tx_ctrl_send_Odometry(kinematic.Sx,kinematic.Sy,kinematic.St,kinematic.Vx,kinematic.Vy,kinematic.Vt);
					}
					handle_heading(0,1);
					if(flag_interrupt==1){
						HAL_Delay(150);
//						flag_interrupt = 1;
					}
//					HAL_Delay(1000);

					if(message_from_sensor.aktuator == 1 && (message_from_sensor.id_data != lastcmd)){
						aktuator_up(aktuator);
						message_from_sensor.aktuator = 0;
						lastcmd = message_from_sensor.id_data;
					}
					else if((message_from_sensor.aktuator == 2) && (message_from_sensor.id_data != lastcmd)){
						aktuator_down(aktuator);
						message_from_sensor.aktuator = 0;
						lastcmd = message_from_sensor.id_data;
					}
					if((message_from_sensor.id_data != lastcmd && (message_from_sensor.x_data != 0 || message_from_sensor.y_data != 0 || message_from_sensor.t_data != 0))){
						lastlength = message_from_sensor.astar_total_length;
						lastcmd = message_from_sensor.id_data;
						flag_interrupt = 1;
						i=-1;
					}


					tx_ctrl_send_Odometry(kinematic.Sx,kinematic.Sy,kinematic.St,kinematic.Vx,kinematic.Vy,kinematic.Vt);
				}

			}
			tx_ctrl_task_done(message_from_sensor.astar_msg_id, &message_from_sensor);
			lastlength = message_from_sensor.astar_total_length;
			lastid = message_from_sensor.astar_msg_id;
		}
		if(message_from_sensor.aktuator == 1 && (message_from_sensor.id_data != lastcmd)){
			aktuator_up(aktuator);
			message_from_sensor.aktuator = 0;
			lastcmd = message_from_sensor.id_data;
		}
		else if((message_from_sensor.aktuator == 2) && (message_from_sensor.id_data != lastcmd)){
			aktuator_down(aktuator);
			message_from_sensor.aktuator = 0;
			lastcmd = message_from_sensor.id_data;
		}
		if((message_from_sensor.id_data != lastcmd && (message_from_sensor.x_data != 0 || message_from_sensor.y_data != 0 || message_from_sensor.t_data != 0))){
			data_sx = kinematic.Sx+message_from_sensor.x_data;
			data_sy = kinematic.Sy+message_from_sensor.y_data;
			data_st = (abs(message_from_sensor.yaw)/100)+message_from_sensor.t_data;

			while(!run_to_point_with_yaw(data_sx,data_sy,0,5)){}
			lastcmd = message_from_sensor.id_data;
			lastlength = message_from_sensor.astar_total_length;
		}
		else{
			agv_stop_all(motor_A, motor_B, motor_C, motor_D);
		}

	//----------------------------- END ASTAR --------------------------------------------//

	    /* USER CODE END WHILE */

	    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 100-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 100-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_UART_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|ENR_D_Pin|ENL_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENR_A_Pin|ENL_A_Pin|ENR_B_Pin|ENL_B_Pin
                          |ENR_C_Pin|ENL_C_Pin|IN1_Pin|IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_UART_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = LED_UART_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin ENR_D_Pin ENL_D_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|ENR_D_Pin|ENL_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENR_A_Pin ENL_A_Pin ENR_B_Pin ENL_B_Pin
                           ENR_C_Pin ENL_C_Pin IN1_Pin IN2_Pin */
  GPIO_InitStruct.Pin = ENR_A_Pin|ENL_A_Pin|ENR_B_Pin|ENL_B_Pin
                          |ENR_C_Pin|ENL_C_Pin|IN1_Pin|IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

bool run_to_point(double sx, double sy, double st, double error){
	if(abs(kinematic.Sx - sx) < error && abs(kinematic.Sy - sy) < error ){
		agv_reset_all(motor_A, motor_B, motor_C, motor_D);
		agv_stop_all(motor_A, motor_B, motor_C, motor_D);
		return true;
	}
	else{
		PIDController_Update(&pid_vx, sx, kinematic.Sx);
		PIDController_Update(&pid_vy, sy, kinematic.Sy);
//		PIDController_Update(&pid_vt, st, kinematic.St);
		agv_reset_all(motor_A, motor_B, motor_C, motor_D);
		agv_inverse_kinematic(pid_vx.out, pid_vy.out, 0, 0, motor_A, motor_B, motor_C, motor_D);
		return false;
	}

}

bool handle_heading(int16_t heading, int16_t error){
	// Sudut 0 - 180 - 0
	if(current_msgid < msgid){
		double degree = 0;
		if(message_from_sensor.yaw > 1800){
			degree = ((-message_from_sensor.yaw)+3600)/10;
			if(abs(heading - degree) < error){
				agv_reset_all(motor_A, motor_B, motor_C, motor_D);
				agv_stop_all(motor_A, motor_B, motor_C, motor_D);
				return true;
			}
			else{
				PIDController_Update(&pid_yaw, heading, degree);
				agv_reset_all(motor_A, motor_B, motor_C, motor_D);
				agv_inverse_kinematic(0, 0, pid_yaw.out, 0, motor_A, motor_B, motor_C, motor_D);
			}
		}
		else{
			degree = ((-message_from_sensor.yaw)+3600)/10;
			if(abs(heading - degree) < error){
				agv_reset_all(motor_A, motor_B, motor_C, motor_D);
				agv_stop_all(motor_A, motor_B, motor_C, motor_D);
				return true;
			}
			else{
				PIDController_Update(&pid_yaw, heading, degree);
				agv_reset_all(motor_A, motor_B, motor_C, motor_D);
				agv_inverse_kinematic(0, 0, -pid_yaw.out, 0, motor_A, motor_B, motor_C, motor_D);
			}
		}
		current_msgid = msgid;
	}
	return false;
}

bool set_heading(int16_t heading, int16_t error){
	if(current_msgid < msgid){
		double degree = (3600 - message_from_sensor.yaw)/10.0;

		if(degree > 180){
			degree = 360 - degree;
		}

		if(abs(heading - degree) < error){
			agv_reset_all(motor_A, motor_B, motor_C, motor_D);
			agv_stop_all(motor_A, motor_B, motor_C, motor_D);
			return true;
		}
		else{
			PIDController_Update(&pid_yaw, heading, degree);
			agv_reset_all(motor_A, motor_B, motor_C, motor_D);

			agv_inverse_kinematic(0, 0, degree > 0 ? pid_yaw.out : -pid_yaw.out, 0, motor_A, motor_B, motor_C, motor_D);
		}
		current_msgid = msgid;
	}
	return false;
}

bool run_to_point_with_yaw(int16_t sx, int16_t sy, uint16_t heading, int16_t error){
//	if(current_msgid < msgid){
		double degree = 0;
		if(abs(kinematic.Sx - sx) < error && abs(kinematic.Sy - sy) < error && abs(degree - heading) < 3){
			agv_reset_all(motor_A, motor_B, motor_C, motor_D);
			agv_stop_all(motor_A, motor_B, motor_C, motor_D);
			return true;
		}
		else{
			if(message_from_sensor.yaw > 1800){
				degree = ((-message_from_sensor.yaw)+3600)/10;
				PIDController_Update(&pid_vx, sx, kinematic.Sx);
				PIDController_Update(&pid_vy, sy, kinematic.Sy);
				PIDController_Update(&pid_yaw, heading, degree);
				agv_reset_all(motor_A, motor_B, motor_C, motor_D);
				agv_inverse_kinematic(pid_vx.out, pid_vy.out, (pid_yaw.out), 0, motor_A, motor_B, motor_C, motor_D);
				return false;
			}
			else if(message_from_sensor.yaw < 1800){
				degree = ((-message_from_sensor.yaw)+3600)/10;
				PIDController_Update(&pid_vx, sx, kinematic.Sx);
				PIDController_Update(&pid_vy, sy, kinematic.Sy);
				PIDController_Update(&pid_yaw, heading, -degree);
				agv_reset_all(motor_A, motor_B, motor_C, motor_D);
				agv_inverse_kinematic(pid_vx.out, pid_vy.out, (pid_yaw.out), 0, motor_A, motor_B, motor_C, motor_D);
				return false;
			}
			else{
				degree = ((-message_from_sensor.yaw)+3600)/10;
				PIDController_Update(&pid_vx, sx, kinematic.Sx);
				PIDController_Update(&pid_vy, sy, kinematic.Sy);
				PIDController_Update(&pid_yaw, heading, 0);
				agv_reset_all(motor_A, motor_B, motor_C, motor_D);
				agv_inverse_kinematic(pid_vx.out, pid_vy.out, (pid_yaw.out), 0, motor_A, motor_B, motor_C, motor_D);
				return false;
			}

		}
//		current_msgid = msgid;
//	}
	return false;
}


bool run_to_point_orientation(double sx, double sy, uint16_t heading, double error){
	if(current_msgid < msgid){
		double degree = 0;
		if(abs(kinematic.Sx - sx) < error && abs(kinematic.Sy - sy) < error && abs(degree - heading) < error){
			agv_reset_all(motor_A, motor_B, motor_C, motor_D);
			agv_stop_all(motor_A, motor_B, motor_C, motor_D);
			return true;
		}
		else{
			if(message_from_sensor.yaw < 0){
				degree = (2*18000 + message_from_sensor.yaw)/100;
			}
			else{
				degree = (message_from_sensor.yaw)/100;
			}
			PIDController_Update(&pid_vx, sx, kinematic.Sx);
			PIDController_Update(&pid_vy, sy, kinematic.Sy);
			PIDController_Update(&pid_yaw, heading, degree);
			agv_reset_all(motor_A, motor_B, motor_C, motor_D);
			agv_inverse_kinematic(pid_vx.out, pid_vy.out, (-pid_yaw.out), 0, motor_A, motor_B, motor_C, motor_D);
			return false;
		}
		current_msgid = msgid;
	}
	return false;
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
