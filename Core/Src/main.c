/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ssd1306/ssd1306.h"
#include "ssd1306/ssd1306_tests.h"
#include <stdbool.h>
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
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
int32_t enc_prev = 0;
int32_t enc_idle_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

#define DRIVER_MOVE_ID 0x01
#define DRIVER_LIFT_ID 0x02

uint8_t bms_uart_buff[100];
uint8_t new_bms_data = 0;

uint8_t bms_detected = 0;
uint8_t smart_bms = 0;

uint8_t bms_jbd_request_msg0[] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
uint8_t bms_jbd_request_msg1[] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
uint8_t bms_smart_request_msg[]  = {0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D};
uint32_t bms_req_time = 0;
int32_t battery_capacity = 0;
uint8_t bms_err = 0;

int32_t start_stop = 0;
int32_t motor_speed = 0;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0,};
uint8_t RxData[8] = {0,};
uint32_t TxMailbox = 0;
volatile uint8_t NewCanMsg = 0;

enum CanGetMsgStatus {
	CAN_GET_MSG_WAIT					=  0,
	CAN_GET_MSG_OK                      =  1,
	CAN_GET_MSG_ERROR                   =  2,
};

typedef struct {
	uint8_t start_msg0;
	uint8_t start_msg1;
	uint8_t shuttle_id;
	uint8_t msg_id;

	uint16_t voltage;
	int16_t current;

	uint32_t remaining_capacity;
	uint16_t nominal_capacity;
	uint16_t n0;

	uint16_t cycles;
	uint16_t date;
	uint16_t balance_low;
	uint16_t balance_high;

	uint16_t protection;
	uint16_t capacity_percent;

	uint8_t version;
	uint8_t MOS_state;
	uint8_t num_of_battery;
	uint8_t num_of_NTC;

	uint16_t temp1;
	uint16_t temp2;

	uint8_t battery_pack;
	uint8_t p0;
	uint8_t p1;
	uint8_t p2;

	uint16_t max_volt;
	uint16_t min_volt;

	uint16_t cell_0;
	uint16_t cell_1;
	uint16_t cell_2;
	uint16_t cell_3;
	uint16_t cell_4;
	uint16_t cell_5;
	uint16_t cell_6;
	uint16_t cell_7;
	uint16_t cell_8;
	uint16_t cell_9;
	uint16_t cell_10;
	uint16_t cell_11;
	uint16_t cell_12;
	uint16_t cell_13;
	uint16_t cell_14;
	uint16_t cell_15;
	uint8_t bms_type; //0-jbd, 1-smart
	uint8_t reservz1;
	uint8_t reservz2;
	uint8_t CS;
} BatteryMsgTypeDef;

typedef struct {
	GPIO_TypeDef *button_port;
	uint16_t button_pin;
	uint8_t short_state;
	uint8_t long_state;
	uint32_t time_key;
} StButtonsTypeDef;

enum BMS_TYPE {
	BMS_NONE = 0,
	BMS_SMART,
	BMS_JBD,
	BMS_MAX,
};

enum MENU {
	MENU_ENC = 0,
	MENU_BMS,
	MENU_DRIVER,
	MENU_MAX,
};

typedef struct {
	int32_t encoder;
	float voltage;
} CanDataRecvTypeDef;

typedef struct {
	uint32_t canId;
	uint32_t canExtId;
	uint32_t canRTR;
	uint8_t canData[8];
} CanDataSendTypeDef;

//CanDataRecvTypeDef canDataRecv;
//CanDataSendTypeDef prevCanData;

int curr_menu = 0;

BatteryMsgTypeDef batteryMsg;
StButtonsTypeDef stButtons[4];

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	uint32_t er = HAL_UART_GetError(huart);
	switch (er) {
		case HAL_UART_ERROR_PE: // ошибка четности
			__HAL_UART_CLEAR_PEFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		case HAL_UART_ERROR_NE:  // шум на линии
			__HAL_UART_CLEAR_NEFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		case HAL_UART_ERROR_FE:  // ошибка фрейма
			__HAL_UART_CLEAR_FEFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		case HAL_UART_ERROR_ORE:  // overrun error
			__HAL_UART_CLEAR_OREFLAG(huart);
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		case HAL_UART_ERROR_DMA:  // ошибка DMA
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			break;
		default:
			break;
		}
	if (huart->Instance == USART1) {
		new_bms_data = 1;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, bms_uart_buff, sizeof(bms_uart_buff));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART1) {
		new_bms_data = 1;
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, bms_uart_buff, sizeof(bms_uart_buff));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		Error_Handler();
	}
	NewCanMsg = CAN_GET_MSG_OK;
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    NewCanMsg = CAN_GET_MSG_ERROR;
}

uint8_t CanMsgRead(CanDataRecvTypeDef *canDataRecv) {

	return 1;
}


uint8_t CanMsgSend(CanDataSendTypeDef *canDataSend) {

	TxHeader.StdId = canDataSend->canId;
	TxHeader.ExtId = canDataSend->canExtId;
	TxHeader.RTR = canDataSend->canRTR;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = 0;
	//globData.can_mutex = 0;
	for (int i = 0; i < sizeof(TxData); i++) {
		TxData[i] = canDataSend->canData[i];
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		//HAL_UART_Transmit(&huart, (uint8_t*) "no_trans\r\n", 10, 100);
		return 0;
	}
	return 1;
}

void Moving() {

	CanDataSendTypeDef canDataSend;

	static int move_axis_en = 0;
	static int lift_axis_en = 0;
	if (start_stop) {
			canDataSend.canExtId = DRIVER_MOVE_ID + 0x06000000;
			canDataSend.canRTR = CAN_RTR_DATA;
			if (!move_axis_en) {
				move_axis_en = 1;
				canDataSend.canData[0] = 0x23;
				canDataSend.canData[1] = 0x0D;
				canDataSend.canData[2] = 0x20;
				canDataSend.canData[3] = 0x01;
				canDataSend.canData[4] = 0x00;
				canDataSend.canData[5] = 0x00;
				canDataSend.canData[6] = 0x00;
				canDataSend.canData[7] = 0x00;
				CanMsgSend(&canDataSend);
				HAL_Delay(10);
			}
			if (!lift_axis_en) {
				lift_axis_en = 1;
				canDataSend.canData[0] = 0x23;
				canDataSend.canData[1] = 0x0D;
				canDataSend.canData[2] = 0x20;
				canDataSend.canData[3] = 0x02;
				canDataSend.canData[4] = 0x00;
				canDataSend.canData[5] = 0x00;
				canDataSend.canData[6] = 0x00;
				canDataSend.canData[7] = 0x00;
				CanMsgSend(&canDataSend);
				HAL_Delay(10);
			}
			static int32_t ch_velocity = 0;
			//if (motor1_speed) { //(l_current_move_comm == MOVE_FORW) {
			//HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)"can_send\r\n", 10, 100);
			canDataSend.canData[0] = 0x23;
			canDataSend.canData[1] = 0x00;
			canDataSend.canData[2] = 0x20;
			canDataSend.canData[3] = 0x01;
			ch_velocity = motor_speed;
			canDataSend.canData[4] = ch_velocity >> 24;
			canDataSend.canData[5] = ch_velocity >> 16;
			canDataSend.canData[6] = ch_velocity >> 8;
			canDataSend.canData[7] = ch_velocity;
			CanMsgSend(&canDataSend);
			HAL_Delay(2);
			//}
			//else if (motor2_speed) {
			canDataSend.canData[0] = 0x23;
			canDataSend.canData[1] = 0x00;
			canDataSend.canData[2] = 0x20;
			canDataSend.canData[3] = 0x02;
			ch_velocity = motor_speed;
			canDataSend.canData[4] = ch_velocity >> 24;
			canDataSend.canData[5] = ch_velocity >> 16;
			canDataSend.canData[6] = ch_velocity >> 8;
			canDataSend.canData[7] = ch_velocity;
			CanMsgSend(&canDataSend);
			//}
		}
		else
		{
			canDataSend.canData[0] = 0x23;
			canDataSend.canData[1] = 0x00;
			canDataSend.canData[2] = 0x20;
			canDataSend.canData[3] = 0x01;
			canDataSend.canData[4] = 0x00;
			canDataSend.canData[5] = 0x00;
			canDataSend.canData[6] = 0x00;
			canDataSend.canData[7] = 0x00;
			CanMsgSend(&canDataSend);
			HAL_Delay(10);
			canDataSend.canData[0] = 0x23;
			canDataSend.canData[1] = 0x00;
			canDataSend.canData[2] = 0x20;
			canDataSend.canData[3] = 0x02;
			canDataSend.canData[4] = 0x00;
			canDataSend.canData[5] = 0x00;
			canDataSend.canData[6] = 0x00;
			canDataSend.canData[7] = 0x00;
			CanMsgSend(&canDataSend);
			move_axis_en = 0;
			lift_axis_en = 0;
		}
		HAL_Delay(5);
}

void get_bms_data()
{
	if (!bms_detected) {
		//HAL_UART_Transmit(&huart1, (uint8_t*)bms_smart_request_msg, sizeof(bms_smart_request_msg), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*)bms_jbd_request_msg0, sizeof(bms_jbd_request_msg0), 100);
		bms_detected = 1;
	}
	else {
		if (smart_bms) HAL_UART_Transmit(&huart1, (uint8_t*)bms_smart_request_msg, sizeof(bms_smart_request_msg), 100);
		else HAL_UART_Transmit(&huart1, (uint8_t*)bms_jbd_request_msg0, sizeof(bms_jbd_request_msg0), 100);
	}
	bms_req_time = HAL_GetTick();
}
void read_bms_uart() {
	if (new_bms_data)
	{
		new_bms_data = 0;
		//bms_err = 0;
		rcGetBattery();
	}
	if (bms_detected && batteryMsg.bms_type == BMS_NONE)
	{
		if (HAL_GetTick() - bms_req_time > 1000) {
			bms_req_time = HAL_GetTick();
			HAL_UART_Transmit(&huart1, (uint8_t*)bms_smart_request_msg, sizeof(bms_smart_request_msg), 100);
			bms_detected = 0;
		}
	}
}

void rcGetBattery() {
	if (bms_uart_buff[0] == 0xDD) smart_bms = 0;
	else if (bms_uart_buff[0] == 0xA5) smart_bms = 1;
	if (smart_bms)
	{
		uint8_t battery_comm = bms_uart_buff[2];
		if (battery_comm == 0x90)
		{
			batteryMsg.bms_type = BMS_SMART;

			batteryMsg.voltage = (bms_uart_buff[4] << 8) + bms_uart_buff[5];
			batteryMsg.current = (bms_uart_buff[8] << 8) + bms_uart_buff[9];
			batteryMsg.capacity_percent = (bms_uart_buff[10] << 8) + bms_uart_buff[11];
			battery_capacity = batteryMsg.capacity_percent/10;
		}
		else if (battery_comm == 0x91)
		{
			batteryMsg.max_volt = (bms_uart_buff[4] << 8) + bms_uart_buff[5];
			batteryMsg.min_volt = (bms_uart_buff[7] << 8) + bms_uart_buff[8];
		}
		else if (battery_comm == 0x92)
		{

		}
		else if (battery_comm == 0x93)
		{
			batteryMsg.remaining_capacity = (bms_uart_buff[8] << 24) +(bms_uart_buff[9] << 16) +(bms_uart_buff[10] << 8) + bms_uart_buff[11];
		}
		else if (battery_comm == 0x94)
		{
			batteryMsg.num_of_battery = bms_uart_buff[4];
			batteryMsg.num_of_NTC = bms_uart_buff[5];
		}
		else if (battery_comm == 0x95)
		{
			if (bms_uart_buff[4] == 0x01)
			{
				batteryMsg.cell_0 = (bms_uart_buff[5] << 8) + bms_uart_buff[6];
				batteryMsg.cell_1 = (bms_uart_buff[7] << 8) + bms_uart_buff[8];
				batteryMsg.cell_2 = (bms_uart_buff[9] << 8) + bms_uart_buff[10];
			}
			else if (bms_uart_buff[4] == 0x02)
			{
				batteryMsg.cell_3 = (bms_uart_buff[5] << 8) + bms_uart_buff[6];
				batteryMsg.cell_4 = (bms_uart_buff[7] << 8) + bms_uart_buff[8];
				batteryMsg.cell_5 = (bms_uart_buff[9] << 8) + bms_uart_buff[10];
				//batteryMsg.cell_3 = (bms_uart_buff[18] << 8) + bms_uart_buff[19];
				//batteryMsg.cell_4 = (bms_uart_buff[20] << 8) + bms_uart_buff[21];
				//batteryMsg.cell_5 = (bms_uart_buff[22] << 8) + bms_uart_buff[23];
			}
			else if (bms_uart_buff[4] == 0x03)
			{
				batteryMsg.cell_6 = (bms_uart_buff[5] << 8) + bms_uart_buff[6];
				batteryMsg.cell_7 = (bms_uart_buff[7] << 8) + bms_uart_buff[8];
				batteryMsg.cell_8 = (bms_uart_buff[9] << 8) + bms_uart_buff[10];
				//batteryMsg.cell_6 = (bms_uart_buff[27] << 8) + bms_uart_buff[28];
				//batteryMsg.cell_7 = (bms_uart_buff[29] << 8) + bms_uart_buff[30];
				//batteryMsg.cell_8 = (bms_uart_buff[31] << 8) + bms_uart_buff[32];
			}
			else if (bms_uart_buff[4] == 0x04)
			{
				batteryMsg.cell_9 = (bms_uart_buff[5] << 8) + bms_uart_buff[6];
				batteryMsg.cell_10 = (bms_uart_buff[7] << 8) + bms_uart_buff[8];
				batteryMsg.cell_11 = (bms_uart_buff[9] << 8) + bms_uart_buff[10];
				//batteryMsg.cell_9 = (bms_uart_buff[34] << 8) + bms_uart_buff[35];
				//batteryMsg.cell_10 = (bms_uart_buff[36] << 8) + bms_uart_buff[37];
				//batteryMsg.cell_11 = (bms_uart_buff[38] << 8) + bms_uart_buff[39];
			}
			else if (bms_uart_buff[4] == 0x05)
			{
				batteryMsg.cell_12 = (bms_uart_buff[5] << 8) + bms_uart_buff[6];
				batteryMsg.cell_13 = (bms_uart_buff[7] << 8) + bms_uart_buff[8];
				batteryMsg.cell_14 = (bms_uart_buff[9] << 8) + bms_uart_buff[10];
				//batteryMsg.cell_12 = (bms_uart_buff[40] << 8) + bms_uart_buff[41];
				//batteryMsg.cell_13 = (bms_uart_buff[42] << 8) + bms_uart_buff[43];
				//batteryMsg.cell_14 = (bms_uart_buff[44] << 8) + bms_uart_buff[45];
			}
			else if (bms_uart_buff[4] == 0x06)
			{
				batteryMsg.cell_15 = (bms_uart_buff[5] << 8) + bms_uart_buff[6];
				//batteryMsg.cell_15 = (bms_uart_buff[47] << 8) + bms_uart_buff[48];
			}
			bms_smart_request_msg[2] = 0x95;
			bms_smart_request_msg[12] = 0x82;
		}
		else if (battery_comm == 0x96)
		{
			batteryMsg.temp1 = bms_uart_buff[5]; //-40 to convert
			batteryMsg.temp2 = bms_uart_buff[6];
		}
		bms_smart_request_msg[2]++;
		bms_smart_request_msg[12]++;
		if (bms_smart_request_msg[2] > 0x96)
		{
			bms_smart_request_msg[2] = 0x90;
			bms_smart_request_msg[12] = 0x7D;
		}
	}
	else
	{
		uint8_t battery_comm = bms_uart_buff[1];
		if (battery_comm == 0x03)
		{
			batteryMsg.bms_type = BMS_JBD;
			batteryMsg.voltage = (bms_uart_buff[4] << 8) + bms_uart_buff[5];
			//batteryMsg.current = 65536 - ((bms_uart_buff[6] << 8) + bms_uart_buff[7]);
			//if (bms_uart_buff[6] & (1 << 8)) batteryMsg.current = -batteryMsg.current;
			batteryMsg.current = 0;//(bms_uart_buff[6] << 8) + bms_uart_buff[7];
			batteryMsg.remaining_capacity = (uint32_t)((bms_uart_buff[8] << 8) + bms_uart_buff[9]);
			batteryMsg.nominal_capacity = (bms_uart_buff[10] << 8) + bms_uart_buff[11];
			batteryMsg.cycles = (bms_uart_buff[12] << 8) + bms_uart_buff[13];
			batteryMsg.date = (bms_uart_buff[14] << 8) + bms_uart_buff[15];
			batteryMsg.balance_low = (bms_uart_buff[16] << 8) + bms_uart_buff[17];
			batteryMsg.balance_high = (bms_uart_buff[18] << 8) + bms_uart_buff[19];
			batteryMsg.protection = (bms_uart_buff[20] << 8) + bms_uart_buff[21];
			batteryMsg.version = bms_uart_buff[22];
			batteryMsg.capacity_percent = (uint16_t)bms_uart_buff[23];
			batteryMsg.MOS_state = bms_uart_buff[24];
			batteryMsg.num_of_battery = bms_uart_buff[25];
			batteryMsg.num_of_NTC = bms_uart_buff[26];
			batteryMsg.temp1 = ((bms_uart_buff[27] << 8) + bms_uart_buff[28]);
			batteryMsg.temp2 = ((bms_uart_buff[29] << 8) + bms_uart_buff[30]);
			battery_capacity = batteryMsg.capacity_percent;

			HAL_UART_Transmit(&huart1, (uint8_t*)bms_jbd_request_msg1, sizeof(bms_jbd_request_msg1), 100);
		}
		else if (battery_comm == 0x04)
		{
			batteryMsg.battery_pack = bms_uart_buff[3];
			batteryMsg.cell_0 = (bms_uart_buff[4] << 8) + bms_uart_buff[5];
			batteryMsg.cell_1 = (bms_uart_buff[6] << 8) + bms_uart_buff[7];
			batteryMsg.cell_2 = (bms_uart_buff[8] << 8) + bms_uart_buff[9];
			batteryMsg.cell_3 = (bms_uart_buff[10] << 8) + bms_uart_buff[11];
			batteryMsg.cell_4 = (bms_uart_buff[12] << 8) + bms_uart_buff[13];
			batteryMsg.cell_5 = (bms_uart_buff[14] << 8) + bms_uart_buff[15];
			batteryMsg.cell_6 = (bms_uart_buff[16] << 8) + bms_uart_buff[17];
			batteryMsg.cell_7 = (bms_uart_buff[18] << 8) + bms_uart_buff[19];
			batteryMsg.cell_8 = (bms_uart_buff[20] << 8) + bms_uart_buff[21];
			batteryMsg.cell_9 = (bms_uart_buff[22] << 8) + bms_uart_buff[23];
			batteryMsg.cell_10 = (bms_uart_buff[24] << 8) + bms_uart_buff[25];
			batteryMsg.cell_11 = (bms_uart_buff[26] << 8) + bms_uart_buff[27];
			batteryMsg.cell_12 = (bms_uart_buff[28] << 8) + bms_uart_buff[29];
			batteryMsg.cell_13 = (bms_uart_buff[30] << 8) + bms_uart_buff[31];
			batteryMsg.cell_14 = (bms_uart_buff[32] << 8) + bms_uart_buff[33];
			batteryMsg.cell_15 = (bms_uart_buff[34] << 8) + bms_uart_buff[35];
		}
	}
}

int32_t unwrap_encoder(uint16_t in, int32_t *prev)
{
    int32_t c32 = (int32_t)in - ENC_HALF_PERIOD;
    int32_t dif = (c32-*prev);

    int32_t mod_dif = ((dif + ENC_HALF_PERIOD) % ENC_ONE_PERIOD) - ENC_HALF_PERIOD;
    if(dif < -ENC_HALF_PERIOD) {
        mod_dif += ENC_ONE_PERIOD;
    }
    int32_t unwrapped = *prev + mod_dif;
    *prev = unwrapped;

    return unwrapped + ENC_HALF_PERIOD;
}

void getEncoder()
{
	int currCounter = __HAL_TIM_GET_COUNTER(&htim4);
	enc_idle_tick = unwrap_encoder(currCounter, &enc_prev);
}

void buttons_Init()
{
	stButtons[0].button_port = BUTTON1_GPIO_Port;
	stButtons[0].button_pin = BUTTON1_Pin;
	stButtons[1].button_port = BUTTON2_GPIO_Port;
	stButtons[1].button_pin = BUTTON2_Pin;
	stButtons[2].button_port = BUTTON3_GPIO_Port;
	stButtons[2].button_pin = BUTTON3_Pin;
	stButtons[3].button_port = BUTTON4_GPIO_Port;
	stButtons[3].button_pin = BUTTON4_Pin;
}

uint8_t short_state = 0;
uint8_t long_state = 0;
uint32_t time_key1 = 0;

void getButton()
{
	for (int i=0; i<4; i++)
	{
		uint32_t ms = HAL_GetTick();
		uint8_t key_state = HAL_GPIO_ReadPin(stButtons[i].button_port, stButtons[i].button_pin);
		if(key_state == 0 && !stButtons[i].short_state && (ms - stButtons[i].time_key) > 50)
		{
			stButtons[i].short_state = 1;
			stButtons[i].long_state = 0;
			stButtons[i].time_key = ms;
		}
		else if(key_state == 0 && !stButtons[i].long_state && (ms - stButtons[i].time_key) > 1000)
		{
			stButtons[i].long_state = 1;
			//long press
			if (i==0)
			{
				curr_menu++;
				if (curr_menu>=MENU_MAX)
				{
					curr_menu = 0;
				}
				if (start_stop)
				{
					start_stop = 0;
				}
			}
		}
		else if(key_state == 1 && stButtons[i].short_state && (ms - stButtons[i].time_key) > 50)
		{
			stButtons[i].short_state = 0;
			stButtons[i].time_key = ms;

		  if(!stButtons[i].long_state)
		  {
			//short press
			  if (curr_menu == MENU_BMS)
			  {
				  if (i == 0)
				  {
					  get_bms_data();
				  }
			  }
			  else if (curr_menu == MENU_DRIVER)
			  {
				  if (i == 0)
				  {
					  start_stop = !start_stop;
					  if (!start_stop)
					  {
						  motor_speed = 0;
					  }
				  }
				  else if (i == 1 && start_stop)
				  {
					  motor_speed += 1000;
					  if (motor_speed>10000) motor_speed = 10000;
				  }
				  else if (i == 2 && start_stop)
				  {
					  motor_speed -= 1000;
				  	  if (motor_speed<-10000) motor_speed = -10000;
				  }
			  }
		  }
		}
	}
}

void menu_update()
{
	SSD1306_COLOR color1 = White;
	SSD1306_COLOR color2 = White;
	SSD1306_COLOR color3 = White;
	if (curr_menu == MENU_ENC)
	{
		char str [12];
		snprintf(str, sizeof str, "%d", (int)enc_idle_tick);
		ssd1306_Fill(Black);
		color1 = Black;
		ssd1306_SetCursor(2, 18);
		ssd1306_WriteString(str, Font_16x26, White);
		if (enc_idle_tick == 0)
		{
			ssd1306_SetCursor(2, 42);
			ssd1306_WriteString("Long press OK", Font_7x10, White);
			ssd1306_SetCursor(2, 53);
			ssd1306_WriteString("to select", Font_7x10, White);
		}
	}
	else if (curr_menu == MENU_BMS)
	{
		ssd1306_Fill(Black);
		color2 = Black;
		if (bms_detected == 0)
		{
			ssd1306_SetCursor(2, 42);
			ssd1306_WriteString("Press OK", Font_7x10, White);
			ssd1306_SetCursor(2, 53);
			ssd1306_WriteString("to start test", Font_7x10, White);
		}
		else
		{
			if (batteryMsg.bms_type == BMS_SMART || batteryMsg.bms_type == BMS_JBD)
			{
				char str [6];
				snprintf(str, sizeof str, "%d", (int)battery_capacity);
				ssd1306_SetCursor(2, 18);
				ssd1306_WriteString(str, Font_16x26, White);
				ssd1306_SetCursor(2, 45);
				if (batteryMsg.bms_type == BMS_SMART )
				{
					ssd1306_WriteString("SMART", Font_11x18, White);
				}
				else
				{
					ssd1306_WriteString("JBD", Font_11x18, White);
				}
			}
			else
			{
				ssd1306_SetCursor(2, 53);
				ssd1306_WriteString("Reading bms data...", Font_7x10, White);
			}
		}
	}
	else if (curr_menu == MENU_DRIVER)
	{
		static uint8_t a = 0;
		ssd1306_Fill(Black);
		if (a)
		{
			color3 = White;
		}
		else
		{
			color3 = Black;
		}
		if (!start_stop)
		{
			ssd1306_SetCursor(2, 42);
			ssd1306_WriteString("OK - start/stop", Font_7x10, White);
			ssd1306_SetCursor(2, 53);
			ssd1306_WriteString("up/down - speed", Font_7x10, White);
			a = 0;
		}
		else
		{
			static uint32_t blink_t = 0;
			char str_act[10] = "Activated!";
			uint32_t ms = HAL_GetTick();
			char str [12];
			snprintf(str, sizeof str, "%d", (int)motor_speed/20);
			ssd1306_SetCursor(2, 18);
			ssd1306_WriteString(str, Font_16x26, White);
			ssd1306_SetCursor(2, 45);
			if (a)
			{
				ssd1306_WriteString(str_act, Font_11x18, White);
			}
			else
			{
				ssd1306_WriteString(str_act, Font_11x18, Black);
			}
			if (ms - blink_t > 100)
			{
				a = !a;
				blink_t = HAL_GetTick();
			}
		}
	}
	ssd1306_SetCursor(2, 1);
	ssd1306_WriteString("Encoder", Font_7x10, color1);
	ssd1306_SetCursor(55, 1);
	ssd1306_WriteString("BMS", Font_7x10, color2);
	ssd1306_SetCursor(80, 1);
	ssd1306_WriteString("Driver", Font_7x10, color3);
	ssd1306_UpdateScreen();
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
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, bms_uart_buff, sizeof(bms_uart_buff));
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  ssd1306_Init();
  buttons_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ssd1306_SetCursor(2, 2);
  ssd1306_Fill(Black);
  ssd1306_WriteString("Start...", Font_16x26, White);
  ssd1306_UpdateScreen();

  while (1)
  {
	  getButton();
	  if(curr_menu == MENU_ENC)
	  {
		  getEncoder();
	  }
	  else if (curr_menu == MENU_BMS)
	  {
		  read_bms_uart();
	  }
	  else if (curr_menu == MENU_DRIVER)
	  {
		  Moving();
	  }
	  menu_update();

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
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  	  sFilterConfig.FilterBank = 0;
      sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
      sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
      sFilterConfig.FilterIdHigh = 0x0000;
      sFilterConfig.FilterIdLow = 0x0000;
      sFilterConfig.FilterMaskIdHigh = 0x0000;
      sFilterConfig.FilterMaskIdLow = 0x0000;
      sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
      sFilterConfig.FilterActivation = ENABLE;
      //sFilterConfig.SlaveStartFilterBank = 14;

      if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
      {
      	Error_Handler();
      }
      if (HAL_CAN_Start(&hcan) != HAL_OK) {
    	Error_Handler();
      }
      if (HAL_CAN_ActivateNotification(&hcan,
    		  CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF
			  	  | CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
    	Error_Handler();
      }
  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OLED_Res_Pin|OLED_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OLED_DC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_Res_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = OLED_Res_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON1_Pin BUTTON2_Pin BUTTON3_Pin BUTTON4_Pin */
  GPIO_InitStruct.Pin = BUTTON1_Pin|BUTTON2_Pin|BUTTON3_Pin|BUTTON4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
