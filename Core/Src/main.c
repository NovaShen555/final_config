/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "memorymap.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "filter.h"
#include "control.h"
#include "driver.h"
#include "motion/motion_types.h"
#include "PTZ.h"
#include "pic/nl.h"
#include "pic/wsnlwcsnl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float x;
  float y;
  float z;
} Speed;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  Car_Constance 15.1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern PID_LocTypeDef Motor_A_PID;
extern PID_LocTypeDef Motor_B_PID;
extern PID_LocTypeDef Z_PID;

char speed_rx_buffer[4096]__attribute__((section(".out")));
char imu_rx_buffer[4096]__attribute__((section(".out")));

char speed_data_buffer[4096];
char imu_data_buffer[4096];


const float ENCODER_RESOLUTION_AB = 500.0f * 4.0f * 30.f;

const float METERS_PER_PULSE_AB = (wheel * 3.1415926535f) / ENCODER_RESOLUTION_AB;//单位cm

Speed_Data Speed_Data_A,Speed_Data_B;

// 新加的正交编码器
Speed_Data Speed_X, Speed_Y;
float last_angle_z = 0.0f; // 上一次的角度
float rotation_speed_x, rotation_speed_y;
float r_y = 10.63f, r_x = 18.3f; // 编码轮到质心的距离 cm
float w ;

Speed Speed_car, Position_car, Imu_Speed;

KalmanFilter speed_x, speed_y, angle_z;
float other_speed_data_x, other_speed_data_y;

float Motor_A_Set,Motor_B_Set;

float z_target = 0.;

float position_target_x = 0, position_target_y = 0;

// jy62
sensor_data_fifo_s accel_data, gyro_data, angle_data;
float accx,accy;

// PTZ
float PTZ_angle_z = 0.0f;
float PTZ_angle_x = 0.0f;

double motor_speed_x, motor_speed_y, motor_speed_z;
bool data_flag = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Move_Transform(double Vx,double Vy,double Vz);
void update_speed(Speed_Data* speed, int tim, float mpp);
void Speed_PID();
void Position_Transform(double Va,double Vb);
void Speed_Int(double dt);
int string2int(int start, int end);
void SpeedHandler();
void PositionHandler();
bool verify_imu_data(int start);
void parse_imu_data(sensor_data_fifo_s* data, int start);
void ImuHandler(uint16_t size);
void CheckPosition();
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_OPAMP2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);


  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim6);

  //imu init
  angle_data.dt = 0.01f;
  angle_data.scale = 1.0f;
  angle_data.samples = 0;
  gyro_data.dt = 0.01f;
  gyro_data.scale = 1.0f;
  gyro_data.samples = 0;
  accel_data.dt = 0.01f;
  accel_data.scale = 1.0f;
  accel_data.samples = 0;

  angle_data.scale = 1.0 / 32768.0f * 180.0f;//degree
  gyro_data.scale = 1.0 / 32768.0f * 2000.0f;//dps
  accel_data.scale = 1.0 / 32768.0f * 16.0f * 9.8f * 100.0f;//cm/s^2

  memset(imu_rx_buffer, 0, sizeof(imu_rx_buffer));

  char z_cali[3] = {0xff,0xaa,0x52};
  // z归零
  HAL_UART_Transmit(&huart4, (uint8_t *)z_cali, sizeof(z_cali), HAL_MAX_DELAY);
  z_cali[2] = 0x67;
  // acc校准
  HAL_UART_Transmit(&huart4, (uint8_t *)z_cali, sizeof(z_cali), HAL_MAX_DELAY);


  Position_car.x = 0;
  Position_car.y = 0;
  Position_car.z = 0;

  Imu_Speed.x = 0;
  Imu_Speed.y = 0;

  kalman_filter_init(&speed_x, 0.01, 0.2);
  kalman_filter_init(&speed_y, 0.01, 0.2);

  kalman_filter_init(&angle_z, 0.01, 0.5);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1,speed_rx_buffer,2048); //1接调试器
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart2,flow_rx_buffer,2048); //2接云台
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart3,imu_rx_buffer,2048);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart4,imu_rx_buffer,2048); //4接imu

  // PTZ back zero
  PTZ_back_zero();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int size = nl_size;
  float (*points)[2] = nl_points;
  while (1)
  {
    for(int i = 0; i <= size; i++) {
      PTZ_angle_z = points[i][0] / 180.0f * PI * 7.; // 转换为角度
      PTZ_angle_x = points[i][1] / 180.0f * PI * 7.; // 转换为角度
      PTZ_update(PTZ_angle_x, PTZ_angle_z);
      HAL_Delay(10);
    }


    char msg[30];

    sprintf(msg, "%d %d %d %d\r\n", (int)(Position_car.x*100), (int)(Position_car.y*100), (int)(Position_car.z*100), (int)(z_target*100));

    // sprintf(msg, "%d %d\r\n", (int)(Speed_Data_A.speed*100), (int)(Speed_Data_B.speed*100));

    HAL_UART_Transmit(&huart1, msg, strlen(msg), HAL_MAX_DELAY);
    HAL_Delay(20);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
// 整车移动量转换为单轮速度  x:�?????????????+�?????????????-  y:�?????????????+�?????????????-  z:�?????????????+�?????????????-
void Move_Transform(double Vx,double Vy,double Vz)
{
  Motor_A_Set=Vy-Vz*Car_Constance;
  Motor_B_Set=Vy+Vz*Car_Constance;
}

// 从指定编码器更新速度到 Speed_Data 结构体
void update_speed(Speed_Data* speed, int tim, float mpp) {

  if (tim == 1) {
    speed->current_count = TIM1->CNT;
  }else if (tim == 2) {
    speed->current_count = TIM2->CNT;
  }


  // speed->current_count = tim.CNT;

  speed->delta_pulses = (speed->current_count - speed->encoder_count_prev);
  speed->encoder_count_prev = speed->current_count;
  //防止计数器超�??????????????????65535
  if(speed->delta_pulses>60000)speed->delta_pulses-=65535;
  if(speed->delta_pulses<-60000)speed->delta_pulses+=65535;
  //计算距离
  speed->delta_distance =  speed->delta_pulses* mpp;
  float dt =0.01;//
  //计算车鿿
  speed->speed = speed->delta_distance / dt;
  speed->distance += speed->delta_distance ;
}

//对轮速进行闭环
void Speed_PID() {


  PID_output_2(Motor_A_Set, Speed_Data_A.speed, &Motor_A_PID, 5000);
  PID_output_2(Motor_B_Set, Speed_Data_B.speed, &Motor_B_PID, 5000);

  // SetMotor(MOTOR_A, (int)Motor_A_PID.output);
  // SetMotor(MOTOR_B, (int)Motor_B_PID.output);

  SetMotor(MOTOR_A, Motor_A_Set);
  SetMotor(MOTOR_B, Motor_B_Set);
}

// 轮速转换为整车速度
void Position_Transform(double Va,double Vb)
{
  float tmp_vy = (Va + Vb) / 2.0;
  float tmp_vx = 0;
  Speed_car.x = kalman_filter_update(&speed_x, tmp_vx);
  Speed_car.y = kalman_filter_update(&speed_y, tmp_vy);
  Speed_car.z = 1.0f/2.0f*(Va-Vb)/Car_Constance;
}

// 速度积分
void Speed_Int(double dt) {
  Position_Transform(Speed_Data_A.speed, Speed_Data_B.speed);

  // 还是融合一下吧
  float ratio = 1.0;
  float tmp_x = Speed_car.x * ratio + (other_speed_data_x) * (1-ratio);
  float tmp_y = Speed_car.y * ratio + (other_speed_data_y) * (1-ratio);
  Speed_car.x = tmp_x;
  Speed_car.y = tmp_y;


  // float tmp_angle_z = kalman_filter_update(&angle_z, angle_data.z[0] * angle_data.scale * 3.1415926535f / 180.0f);
  float tmp_angle_z = angle_data.z[0] * angle_data.scale * 3.1415926535f / 180.0f;
  if (tmp_angle_z > 2.0f * PI) tmp_angle_z -= 2.0f * PI;
  Position_car.z = tmp_angle_z; // 角度转弧度

  // 限制范围
  if (tmp_angle_z - last_angle_z > PI) {
    last_angle_z += 2*PI;
  } else if (tmp_angle_z - last_angle_z < -PI) {
    last_angle_z -= 2*PI;
  }


  if (isnan(Position_car.x)) Position_car.x = 0;
  if (isnan(Position_car.y)) Position_car.y = 0;

  // 积分
  Position_car.x += (Speed_car.x*cosf(Position_car.z)+Speed_car.y*sinf(Position_car.z))*dt;
  Position_car.y += (-Speed_car.x*sinf(Position_car.z)+Speed_car.y*cosf(Position_car.z))*dt;
}

// 整数转换函数
int string2int(int start, int end) {
  int result = 0;
  if (speed_data_buffer[start] == '-') {
    start++;
    while (start < end && speed_data_buffer[start] == '0') start++;
    return -string2int(start, end);
  }
  for (int i = start; i <= end; i++) {
    if (speed_data_buffer[i] >= '0' && speed_data_buffer[i] <= '9') {
      result = result * 10 + (speed_data_buffer[i] - '0');
    }
  }
  return result;
}

// 接收串口数据包，解析成速度
void SpeedHandler() {
  // HAL_UART_Transmit(&huart1, (uint8_t *)data_buffer, strlen(data_buffer), HAL_MAX_DELAY);
  int idx0 = 0;
  while (speed_data_buffer[idx0] != ':') idx0++;
  int idx1 = idx0 + 1;
  while (speed_data_buffer[idx1] != ',') idx1++;
  int idx2 = idx1 + 1;
  while (speed_data_buffer[idx2] != ',') idx2++;
  int idx3 = idx2 + 1;
  while (speed_data_buffer[idx3] != ',') idx3++;
  int idx4 = idx3 + 1;
  while (speed_data_buffer[idx4] != ',') idx4++;
  int idx5 = idx4 + 1;
  while ((speed_data_buffer[idx5] != '\r')&&(speed_data_buffer[idx5] !='\n')) idx5++;

  motor_speed_y = -string2int(idx0+1, idx1-1) / 1.50;
  z_target += string2int(idx2+1, idx3-1) / 3000.0;
  if (z_target > 2.0f * PI) z_target -= 2.0f * PI;
  if (z_target < 0) z_target += 2.0f * PI;

  PTZ_angle_z += string2int(idx3+1, idx4-1) / 5000.0f;
  if (PTZ_angle_z > PI) PTZ_angle_z = PI;
  if (PTZ_angle_z < -PI) PTZ_angle_z = -PI;
  PTZ_angle_x += string2int(idx4+1, idx5-1) / 5000.0f;
  if (PTZ_angle_x > PI) PTZ_angle_x = PI;
  if (PTZ_angle_x < -PI) PTZ_angle_x = -PI;
  PTZ_set_angle(0x02, PTZ_angle_z);
  PTZ_set_angle(0x01, PTZ_angle_x);
}

// 接收串口数据包，解析成位置
void PositionHandler() {
  int idx0 = 0;
  while (speed_data_buffer[idx0] != ':') idx0++;
  int idx1 = idx0 + 1;
  while (speed_data_buffer[idx1] != ',') idx1++;
  int idx2 = idx1 + 1;
  while ((speed_data_buffer[idx2] != '\r')&&(speed_data_buffer[idx2] !='\n')) idx2++;

  position_target_x = string2int(idx0+1, idx1-1) / 100.0;
  position_target_y = string2int(idx1+1, idx2-1) / 100.0;
}

bool verify_imu_data(int start) {
  char tmp = 0;
  for (int i=0; i<10; i++) {
    tmp += imu_data_buffer[start+i];
  }
  return tmp == imu_data_buffer[start+10];
}

// 解析IMU数据
int16_t parse_tmp[3] = {0};
void parse_imu_data(sensor_data_fifo_s* data, int start) {

  for (int i=0;i<3;i++) {

    int16_t tmp1 = imu_data_buffer[start+2+i*2];
    int16_t tmp2 = imu_data_buffer[start+3+i*2];
    int16_t tmp3 = tmp2<<8 | tmp1;

    parse_tmp[i] = imu_data_buffer[start+3+i*2] << 8 | imu_data_buffer[start+2+i*2];
  }

  data->x[data->samples] = parse_tmp[0];
  data->y[data->samples] = parse_tmp[1];
  data->z[data->samples] = parse_tmp[2];
  data->samples++;

}

// 串口回调处理器
void ImuHandler(uint16_t size) {

  accel_data.samples = 0;
  gyro_data.samples = 0;
  angle_data.samples = 0;

  int max_size = (size > 10)?size-10:size;
  for (int i=0; i<max_size; i++) {
    if ((uint16_t)imu_data_buffer[i] == 85) {
      if (verify_imu_data(i)) {
        if (imu_data_buffer[i+1] == 0x51) {
          //加�?�度
          parse_imu_data(&accel_data, i);
        }
        else if (imu_data_buffer[i+1] == 0x52) {
          //角�?�度
          parse_imu_data(&gyro_data, i);
        }
        else if (imu_data_buffer[i+1] == 0x53) {
          //角度
          parse_imu_data(&angle_data, i);
        }
      }
    }
  }
}

// 检查位置是否到达目标位置
void CheckPosition() {
  float distance = sqrtf(powf(Position_car.x - position_target_x, 2) + powf(Position_car.y - position_target_y, 2));


  if (distance < 5) {
    motor_speed_y = 0;
  } else {
    motor_speed_y = 200;
    z_target = atan2(position_target_x - Position_car.x, position_target_y - Position_car.y);
  }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==TIM6) {

    if (data_flag) SpeedHandler(),data_flag = false;
    // if (data_flag) PositionHandler(),data_flag = false;
    // CheckPosition();

    update_speed(&Speed_Data_A, 1, METERS_PER_PULSE_AB);
    update_speed(&Speed_Data_B, 2, METERS_PER_PULSE_AB);

    Speed_Y.speed *= -1.0f; // 反向

    Speed_Int(0.01f);


    Speed_PID();

    if (Position_car.z - z_target > PI) Position_car.z -= 2.0f * PI;
    if (Position_car.z - z_target < -PI) Position_car.z += 2.0f * PI;

    if (fabs(Position_car.z - z_target) > 0.02) {
      PID_output(z_target, Position_car.z, &Z_PID, 2000);
      motor_speed_z = Z_PID.output;
    }
    else {
      motor_speed_z = 0;
    }

    if (motor_speed_z > 30)motor_speed_z = 30;
    if (motor_speed_z < -30)motor_speed_z = -30;
    Move_Transform(motor_speed_x, motor_speed_y, motor_speed_z);
  }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart->Instance == USART1) {
    strcpy(speed_data_buffer, speed_rx_buffer);
    data_flag = true;
    memset(speed_rx_buffer, 0, sizeof(speed_rx_buffer));
  }
  if (huart->Instance == USART2) {

  }
  if (huart->Instance == USART3) {

  }
  if (huart->Instance == UART4) {
    memcpy(imu_data_buffer, imu_rx_buffer, Size);
    ImuHandler(Size);
    memset(imu_rx_buffer, 0, sizeof(imu_rx_buffer));
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
