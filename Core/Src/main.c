/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Automotive Telematics & Gateway (Phase 5)
  *                   Features: Dual bxCAN, J1939 Powertrain, FreeRTOS CMSIS_V2,
  *                             UDS ISO 14229 (0x22, 0x27, 0x14, 0x19),
  *                             ISO-TP ISO 15765-2 Multi-frame VIN Reassembly,
  *                             Flash Sector 7 Crash Freeze Frame Data Logger.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */
osSemaphoreId_t canSemHandle;
osThreadId_t powertrainTaskHandle;
osThreadId_t gatewayRxTaskHandle;
osThreadId_t telematicsTaskHandle;

void StartPowertrainTask(void *argument);
void StartGatewayRxTask(void *argument);
void StartTelematicsTask(void *argument);

typedef struct __attribute__((packed)){
	uint16_t rpm;
	uint8_t speed;
	uint8_t coolant_temp;
	uint8_t throttle;
	uint8_t mil_lamp: 1;
	uint8_t overheat: 1;
	uint8_t drive_mode: 2;
	uint8_t brake_pedal: 1;
	uint8_t reserved: 3;
	uint16_t dtc_code;
} Powertrain_Data_t;

typedef union {
	Powertrain_Data_t data;
	uint8_t raw_bytes[8];
} Powertrain_Packet_u;

Powertrain_Packet_u tx_vehicle;
Powertrain_Packet_u rx_vehicle;

CAN_TxHeaderTypeDef tx_hdr_powertrain;
uint32_t tx_mailbox_can1;

// ============================================================================
// UDS (ISO 14229) & ISO-TP (ISO 15765-2) DEFINITIONS
// ============================================================================
const char VEHICLE_VIN[17] = "VF8VN2026TEST001"; // 17 ký tự VIN chuẩn quốc tế

#define UDS_SID_READ_DID         0x22  // Read Data By Identifier
#define UDS_SID_SECURITY_ACCESS  0x27  // Security Access
#define UDS_SID_READ_DTC         0x19  // Read DTC Information / Freeze Frame
#define UDS_SID_CLEAR_DTC        0x14  // Clear Diagnostic Information
#define UDS_DID_VIN              0xF190 // DID chuẩn số khung VIN

CAN_TxHeaderTypeDef tx_hdr_uds_tester; // ID 0x7DF (Tester phát lệnh)
CAN_TxHeaderTypeDef tx_hdr_uds_ecu;    // ID 0x7E8 (ECU phát trả lời)
uint32_t            tx_mailbox_uds;

uint8_t isotp_rx_buffer[32];
uint8_t isotp_rx_index = 0;
uint8_t isotp_total_len = 0;

char last_vin_received[20] = "CHUA DOC (0xF190)";
char last_security_status[32] = "LOCKED (0x27)";
uint16_t ecu_security_seed = 0;
uint8_t  ecu_security_unlocked = 0;

// ============================================================================
// CRASH DATA LOGGER & FLASH SECTOR 7 DEFINITIONS
// ============================================================================
#define FLASH_CRASH_SECTOR      FLASH_SECTOR_7
#define FLASH_CRASH_ADDR        0x08060000U
#define FLASH_MAGIC_KEY         0xA55A1234U

typedef struct __attribute__((packed)) {
    uint32_t magic_key;     // 0xA55A1234: Co du lieu hop den
    uint32_t timestamp_ms;  // Thoi diem va cham
    uint16_t dtc_code;      // 0x0115 (P0115)
    uint8_t  speed;         // km/h
    uint16_t rpm;           // RPM
    uint8_t  throttle;      // %
    uint8_t  coolant_temp;  // do C
    uint8_t  brake_pedal;   // 1
    uint8_t  crash_severity;// 2 = Critical
} Crash_FreezeFrame_t;

Crash_FreezeFrame_t saved_crash_data;
uint8_t has_saved_crash = 0;
char last_crash_str[64] = "FLASH EMPTY (READY)";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// ============================================================================
// FLASH SECTOR 7 DRIVER (NON-VOLATILE CRASH DATA STORAGE)
// ============================================================================
void Flash_Save_CrashData(Crash_FreezeFrame_t *data)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = FLASH_CRASH_SECTOR;
    EraseInitStruct.NbSectors    = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) == HAL_OK)
    {
        uint32_t *src = (uint32_t *)data;
        uint32_t words_to_write = (sizeof(Crash_FreezeFrame_t) + 3) / 4;
        for (uint32_t i = 0; i < words_to_write; i++)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_CRASH_ADDR + (i * 4), src[i]);
        }
    }

    HAL_FLASH_Lock();
}

void Flash_Clear_CrashData(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector       = FLASH_CRASH_SECTOR;
    EraseInitStruct.NbSectors    = 1;

    HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

    HAL_FLASH_Lock();
}

uint8_t Flash_Read_CrashData(Crash_FreezeFrame_t *out_data)
{
    Crash_FreezeFrame_t *flash_ptr = (Crash_FreezeFrame_t *)FLASH_CRASH_ADDR;
    if (flash_ptr->magic_key == FLASH_MAGIC_KEY)
    {
        memcpy(out_data, flash_ptr, sizeof(Crash_FreezeFrame_t));
        return 1;
    }
    return 0;
}

void CAN_Filter_Config(void)
{
	CAN_FilterTypeDef sFilterConfig;

	// Bank 0 cho CAN1: Cho qua tat ca (Mask = 0) de nhan ca 0x7DF / 0x7E0
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

	// Bank 14 cho CAN2: Cho qua tat ca (Mask = 0) de nhan ca Powertrain 0x0CF00400 va UDS Response 0x7E8
	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000; // Mo Mask = 0 de CAN2 nhan duoc ca 0x7E8 va 0x0CF00400
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_hdr;
  uint8_t rx_data[8];

  // ------------------------------------------------------------------------
  // 1. CAN1 (ENGINE ECU) NHAN LENH CHUAN DOAN TU TESTER (ID 0x7DF HOAC 0x7E0)
  // ------------------------------------------------------------------------
  if (hcan->Instance == CAN1)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, rx_data) == HAL_OK)
    {
      if (rx_hdr.IDE == CAN_ID_STD && (rx_hdr.StdId == 0x7DF || rx_hdr.StdId == 0x7E0))
      {
        uint8_t pci_type = (rx_data[0] & 0xF0) >> 4;

        // a) SINGLE FRAME (0x0...): Tester gui yeu cau UDS
        if (pci_type == 0x00)
        {
          uint8_t sid = rx_data[1];

          // DICH VU 0x22: READ DATA BY IDENTIFIER (DOC SO VIN)
          if (sid == UDS_SID_READ_DID)
          {
            uint16_t did = ((uint16_t)rx_data[2] << 8) | rx_data[3];
            if (did == UDS_DID_VIN)
            {
              // So VIN 17 byte + 3 byte header (0x62, 0xF1, 0x90) = 20 byte!
              // Du lieu > 7 byte -> Bat buoc phat First Frame (FF) theo chuan ISO-TP
              uint8_t ff_data[8];
              ff_data[0] = 0x10; // PCI: First Frame
              ff_data[1] = 0x14; // Tong kich thuoc: 20 byte (0x014)
              ff_data[2] = 0x62; // Positive Response (0x22 + 0x40)
              ff_data[3] = 0xF1;
              ff_data[4] = 0x90;
              ff_data[5] = VEHICLE_VIN[0]; // 'V'
              ff_data[6] = VEHICLE_VIN[1]; // 'F'
              ff_data[7] = VEHICLE_VIN[2]; // '8'
              HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, ff_data, &tx_mailbox_uds);
            }
          }
          // DICH VU 0x27: SECURITY ACCESS
          else if (sid == UDS_SID_SECURITY_ACCESS)
          {
            uint8_t sub_func = rx_data[2];
            if (sub_func == 0x01) // Xin Seed
            {
              ecu_security_seed = 0x5A3C;
              uint8_t seed_resp[8] = {0x04, 0x67, 0x01, (uint8_t)(ecu_security_seed >> 8), (uint8_t)(ecu_security_seed & 0xFF), 0xAA, 0xAA, 0xAA};
              HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, seed_resp, &tx_mailbox_uds);
            }
            else if (sub_func == 0x02) // Gui Key
            {
              uint16_t key_rx = ((uint16_t)rx_data[3] << 8) | rx_data[4];
              uint16_t key_exp = ecu_security_seed ^ 0xABCD; // Thuat toan: Key = Seed XOR 0xABCD
              if (key_rx == key_exp)
              {
                ecu_security_unlocked = 1;
                uint8_t key_ok[8] = {0x02, 0x67, 0x02, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
                HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, key_ok, &tx_mailbox_uds);
              }
              else
              {
                ecu_security_unlocked = 0;
                uint8_t key_fail[8] = {0x03, 0x7F, 0x27, 0x35, 0xAA, 0xAA, 0xAA, 0xAA}; // NRC 0x35 = Invalid Key
                HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, key_fail, &tx_mailbox_uds);
              }
            }
          }
          // DICH VU 0x14: CLEAR DIAGNOSTIC INFORMATION (XOA DTC & XOA HOP DEN FLASH)
          else if (sid == UDS_SID_CLEAR_DTC)
          {
            tx_vehicle.data.dtc_code = 0x0000;
            tx_vehicle.data.mil_lamp = 0;
            Flash_Clear_CrashData();
            has_saved_crash = 0;
            strcpy(last_crash_str, "FLASH EMPTY (READY)");

            uint8_t clr_resp[8] = {0x01, 0x54, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // 0x14 + 0x40 = 0x54
            HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, clr_resp, &tx_mailbox_uds);
          }
          // DICH VU 0x19: READ DTC & FREEZE FRAME
          else if (sid == UDS_SID_READ_DTC)
          {
            uint8_t sub_func = rx_data[2];
            if (sub_func == 0x02) // Read DTC By Status Mask
            {
              if (tx_vehicle.data.dtc_code != 0)
              {
                uint8_t dtc_resp[8] = {0x06, 0x59, 0x02, 0xFF, (uint8_t)(tx_vehicle.data.dtc_code >> 8), (uint8_t)(tx_vehicle.data.dtc_code & 0xFF), 0x24, 0xAA};
                HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, dtc_resp, &tx_mailbox_uds);
              }
              else
              {
                uint8_t no_dtc[8] = {0x03, 0x59, 0x02, 0x00, 0xAA, 0xAA, 0xAA, 0xAA};
                HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, no_dtc, &tx_mailbox_uds);
              }
            }
            else if (sub_func == 0x04) // Read DTC Snapshot / Freeze Frame
            {
              if (Flash_Read_CrashData(&saved_crash_data))
              {
                uint8_t ff_resp[8] = {
                  0x07, 0x59, 0x04,
                  saved_crash_data.speed,
                  (uint8_t)(saved_crash_data.rpm >> 8),
                  (uint8_t)(saved_crash_data.rpm & 0xFF),
                  saved_crash_data.throttle,
                  saved_crash_data.coolant_temp
                };
                HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, ff_resp, &tx_mailbox_uds);
              }
              else
              {
                uint8_t no_ff[8] = {0x03, 0x7F, 0x19, 0x31, 0xAA, 0xAA, 0xAA, 0xAA}; // NRC 0x31 = Request Out Of Range
                HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, no_ff, &tx_mailbox_uds);
              }
            }
          }
        }
        // b) FLOW CONTROL (0x3...): Tester dong y cho ECU ban tiep Consecutive Frames
        else if (pci_type == 0x03)
        {
          // Phat Consecutive Frame 1: 7 byte (VIN[3..9])
          uint8_t cf1[8];
          cf1[0] = 0x21; // CF Sequence 1
          memcpy(&cf1[1], &VEHICLE_VIN[3], 7);
          HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, cf1, &tx_mailbox_uds);

          // Phat Consecutive Frame 2: 7 byte con lai (VIN[10..16])
          uint8_t cf2[8];
          cf2[0] = 0x22; // CF Sequence 2
          memcpy(&cf2[1], &VEHICLE_VIN[10], 7);
          HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_uds_ecu, cf2, &tx_mailbox_uds);
        }
      }
    }
  }
  // ------------------------------------------------------------------------
  // 2. CAN2 (GATEWAY / TESTER) NHAN PHAN HOI TU ECU (0x0CF00400 HOAC 0x7E8)
  // ------------------------------------------------------------------------
  else if (hcan->Instance == CAN2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_hdr, rx_data) == HAL_OK)
    {
      // a) Goi tin Powertrain dinh ky J1939 ExtID 0x0CF00400
      if (rx_hdr.IDE == CAN_ID_EXT && rx_hdr.ExtId == 0x0CF00400)
      {
        memcpy(rx_vehicle.raw_bytes, rx_data, 8);
        if (canSemHandle != NULL)
        {
          osSemaphoreRelease(canSemHandle);
        }
      }
      // b) Goi tin UDS / ISO-TP Response StdID 0x7E8
      else if (rx_hdr.IDE == CAN_ID_STD && rx_hdr.StdId == 0x7E8)
      {
        uint8_t pci_type = (rx_data[0] & 0xF0) >> 4;

        // FIRST FRAME (0x1...): Nhan mau dau tien cua so VIN
        if (pci_type == 0x01)
        {
          isotp_total_len = (((uint16_t)rx_data[0] & 0x0F) << 8) | rx_data[1]; // 20 byte
          isotp_rx_index = 0;
          memcpy(&isotp_rx_buffer[0], &rx_data[2], 6); // Luu 6 byte dau
          isotp_rx_index = 6;

          // Tester lap tuc ban Flow Control (FC: Clear to Send) sang ECU
          uint8_t fc_frame[8] = {0x30, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
          HAL_CAN_AddTxMessage(&hcan2, &tx_hdr_uds_tester, fc_frame, &tx_mailbox_uds);
        }
        // CONSECUTIVE FRAME 1 (0x21)
        else if (rx_data[0] == 0x21)
        {
          memcpy(&isotp_rx_buffer[isotp_rx_index], &rx_data[1], 7);
          isotp_rx_index += 7;
        }
        // CONSECUTIVE FRAME 2 (0x22)
        else if (rx_data[0] == 0x22)
        {
          memcpy(&isotp_rx_buffer[isotp_rx_index], &rx_data[1], 7);
          isotp_rx_index += 7;

          // Ghep du 17 ky tu so VIN vao chuoi!
          memcpy(last_vin_received, &isotp_rx_buffer[3], 17);
          last_vin_received[17] = '\0';
        }
        // SINGLE FRAME (0x0...): Phan hoi UDS
        else if (pci_type == 0x00)
        {
          // Phan hoi Security Access (0x67)
          if (rx_data[1] == 0x67 && rx_data[2] == 0x01) // Nhan Seed
          {
            uint16_t seed = ((uint16_t)rx_data[3] << 8) | rx_data[4];
            uint16_t key = seed ^ 0xABCD; // Tester tu dong giai toan Key = Seed XOR 0xABCD
            uint8_t send_key[8] = {0x04, 0x27, 0x02, (uint8_t)(key >> 8), (uint8_t)(key & 0xFF), 0xAA, 0xAA, 0xAA};
            HAL_CAN_AddTxMessage(&hcan2, &tx_hdr_uds_tester, send_key, &tx_mailbox_uds);
          }
          else if (rx_data[1] == 0x67 && rx_data[2] == 0x02) // Bao mo khoa thanh cong
          {
            strcpy(last_security_status, "UNLOCKED (0x27 OK)");
          }
          else if (rx_data[1] == 0x7F && rx_data[2] == 0x27) // Bao that bai
          {
            strcpy(last_security_status, "LOCKED (NRC 0x35)");
          }
          // Phan hoi Clear DTC (0x54)
          else if (rx_data[1] == 0x54)
          {
            has_saved_crash = 0;
            strcpy(last_crash_str, "FLASH EMPTY (READY)");
          }
          // Phan hoi Read Freeze Frame (0x59 0x04)
          else if (rx_data[1] == 0x59 && rx_data[2] == 0x04)
          {
            uint8_t spd = rx_data[3];
            uint16_t rpm = ((uint16_t)rx_data[4] << 8) | rx_data[5];
            uint8_t thr = rx_data[6];
            uint8_t temp = rx_data[7];
            has_saved_crash = 1;
            snprintf(last_crash_str, sizeof(last_crash_str), "CRASH: %ukm/h, %uRPM, %u%%, %uC", spd, rpm, thr, temp);
          }
        }
      }
    }
  }
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
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

  CAN_Filter_Config();

  // Doc kiem tra hop den Flash Sector 7 luc khoi dong
  if (Flash_Read_CrashData(&saved_crash_data))
  {
    has_saved_crash = 1;
    snprintf(last_crash_str, sizeof(last_crash_str), "CRASH: %ukm/h, %uRPM, P%04X", saved_crash_data.speed, saved_crash_data.rpm, saved_crash_data.dtc_code);
  }
  else
  {
    has_saved_crash = 0;
    strcpy(last_crash_str, "FLASH EMPTY (READY)");
  }

  if(HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
  if(HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();

  if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
  if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();

  tx_hdr_powertrain.ExtId = 0x0CF00400;
  tx_hdr_powertrain.IDE = CAN_ID_EXT;
  tx_hdr_powertrain.RTR = CAN_RTR_DATA;
  tx_hdr_powertrain.DLC = 8;
  tx_hdr_powertrain.TransmitGlobalTime = DISABLE;

  tx_vehicle.data.rpm = 800;
  tx_vehicle.data.speed = 0;
  tx_vehicle.data.coolant_temp = 85 + 40;
  tx_vehicle.data.throttle = 15;
  tx_vehicle.data.drive_mode = 1;
  tx_vehicle.data.mil_lamp = 0;
  tx_vehicle.data.overheat = 0;
  tx_vehicle.data.brake_pedal = 0;
  tx_vehicle.data.dtc_code = 0x0000;

  // Khoi tao Header UDS Tester (0x7DF) va ECU (0x7E8)
  tx_hdr_uds_tester.StdId = 0x7DF;
  tx_hdr_uds_tester.IDE   = CAN_ID_STD;
  tx_hdr_uds_tester.RTR   = CAN_RTR_DATA;
  tx_hdr_uds_tester.DLC   = 8;
  tx_hdr_uds_tester.TransmitGlobalTime = DISABLE;

  tx_hdr_uds_ecu.StdId = 0x7E8;
  tx_hdr_uds_ecu.IDE   = CAN_ID_STD;
  tx_hdr_uds_ecu.RTR   = CAN_RTR_DATA;
  tx_hdr_uds_ecu.DLC   = 8;
  tx_hdr_uds_ecu.TransmitGlobalTime = DISABLE;

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  canSemHandle = osSemaphoreNew(1, 0, NULL);
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  const osThreadAttr_t powertrain_attr = {
     .name = "Powertrain",
     .stack_size = 256 * 4,
     .priority = (osPriority_t) osPriorityAboveNormal, // Ưu tiên cao
   };
   const osThreadAttr_t gateway_attr = {
     .name = "GatewayRx",
     .stack_size = 256 * 4,
     .priority = (osPriority_t) osPriorityRealtime,    // Ưu tiên thời gian thực
   };
   const osThreadAttr_t telematics_attr = {
     .name = "Telematics",
     .stack_size = 512 * 4,                           // Stack lớn hơn cho printf
     .priority = (osPriority_t) osPriorityNormal,      // Ưu tiên bình thường
   };
   // Tạo 3 Task chạy song song
   powertrainTaskHandle = osThreadNew(StartPowertrainTask, NULL, &powertrain_attr);
   gatewayRxTaskHandle  = osThreadNew(StartGatewayRxTask, NULL, &gateway_attr);
   telematicsTaskHandle = osThreadNew(StartTelematicsTask, NULL, &telematics_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  __HAL_RCC_PWR_CLK_ENABLE();

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void StartPowertrainTask(void *argument)
{
	for (;;)
	{
		 if (tx_vehicle.data.brake_pedal == 1) {
		      if (tx_vehicle.data.speed > 5) tx_vehicle.data.speed -= 5;
		      else tx_vehicle.data.speed = 0;
		      if (tx_vehicle.data.rpm > 1000) tx_vehicle.data.rpm -= 150;
		      else tx_vehicle.data.rpm = 800;
		      tx_vehicle.data.throttle = 0;
		    } else {
		      tx_vehicle.data.throttle = 25 + (tx_vehicle.data.drive_mode * 25);
		      tx_vehicle.data.rpm += 40; // Tăng ga mượt mà hơn với chu kỳ 50ms
		      if (tx_vehicle.data.rpm > 4500) tx_vehicle.data.rpm = 1200;
		      tx_vehicle.data.speed = tx_vehicle.data.rpm / 40;
		 }
		 HAL_CAN_AddTxMessage(&hcan1, &tx_hdr_powertrain, tx_vehicle.raw_bytes, &tx_mailbox_can1);
		 osDelay(50);
	}
}

void StartGatewayRxTask(void *argument)
{
  for (;;)
  {
    // Ngủ vô tận chờ ngắt CAN2 nhả Semaphore (Tiêu thụ 0% CPU khi không có frame)
    if (osSemaphoreAcquire(canSemHandle, osWaitForever) == osOK)
    {
      // Khi thức dậy: Frame Powertrain đã được nhận an toàn
    }
  }
}

void StartTelematicsTask(void *argument)
{
  for (;;)
  {
    // 1. Kiểm tra phím bấm từ bàn phím PC
    uint8_t key = 0;
    if (HAL_UART_Receive(&huart1, &key, 1, 0) == HAL_OK)
    {
      if (key == 'b' || key == 'B') tx_vehicle.data.brake_pedal = !tx_vehicle.data.brake_pedal;
      else if (key == 'e' || key == 'E') {
        tx_vehicle.data.mil_lamp = 1;
        tx_vehicle.data.dtc_code = 0x0115;
      }
      else if (key == 'c' || key == 'C') {
        // CAN2 phat lenh UDS 0x14 xoa toan bo DTC va xoa Flash Sector 7
        uint8_t clr_req[8] = {0x04, 0x14, 0xFF, 0xFF, 0xFF, 0xAA, 0xAA, 0xAA};
        HAL_CAN_AddTxMessage(&hcan2, &tx_hdr_uds_tester, clr_req, &tx_mailbox_uds);
      }
      else if (key == 'm' || key == 'M') {
        tx_vehicle.data.drive_mode = (tx_vehicle.data.drive_mode + 1) % 3;
      }
      else if (key == 'v' || key == 'V') {
        // CAN2 (Tester) phát lệnh UDS đọc số VIN 17-byte (Service 0x22, DID 0xF190)
        uint8_t vin_req[8] = {0x03, 0x22, 0xF1, 0x90, 0xAA, 0xAA, 0xAA, 0xAA};
        HAL_CAN_AddTxMessage(&hcan2, &tx_hdr_uds_tester, vin_req, &tx_mailbox_uds);
      }
      else if (key == 's' || key == 'S') {
        // CAN2 (Tester) phát lệnh UDS yêu cầu Security Access (Service 0x27, Sub-func 0x01 Request Seed)
        uint8_t sec_req[8] = {0x02, 0x27, 0x01, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA};
        HAL_CAN_AddTxMessage(&hcan2, &tx_hdr_uds_tester, sec_req, &tx_mailbox_uds);
      }
      else if (key == 'r' || key == 'R') {
        // CAN2 (Tester) phat lenh UDS 0x19 0x04 doc Freeze Frame tu ECU
        uint8_t req_ff[8] = {0x04, 0x19, 0x04, 0x01, 0x15, 0xAA, 0xAA, 0xAA};
        HAL_CAN_AddTxMessage(&hcan2, &tx_hdr_uds_tester, req_ff, &tx_mailbox_uds);
      }
      else if (key == 'k' || key == 'K') {
        // Kich hoat va cham khẩn cap: Phanh gap, bao loi dong co P0115
        tx_vehicle.data.brake_pedal = 1;
        tx_vehicle.data.mil_lamp = 1;
        tx_vehicle.data.dtc_code = 0x0115;

        // Chup anh thoi diem va cham (Freeze Frame Snapshot)
        Crash_FreezeFrame_t crash;
        crash.magic_key = FLASH_MAGIC_KEY;
        crash.timestamp_ms = HAL_GetTick();
        crash.dtc_code = 0x0115;
        crash.speed = tx_vehicle.data.speed;
        crash.rpm = tx_vehicle.data.rpm;
        crash.throttle = tx_vehicle.data.throttle;
        crash.coolant_temp = (tx_vehicle.data.coolant_temp > 40) ? (tx_vehicle.data.coolant_temp - 40) : 0;
        crash.brake_pedal = 1;
        crash.crash_severity = 2; // Critical Crash Event

        // Ghi tuc thi vao Flash Sector 7
        Flash_Save_CrashData(&crash);
        memcpy(&saved_crash_data, &crash, sizeof(Crash_FreezeFrame_t));
        has_saved_crash = 1;
        snprintf(last_crash_str, sizeof(last_crash_str), "CRASH: %ukm/h, %uRPM, P0115", crash.speed, crash.rpm);
      }
    }

    // 2. Đọc bộ đếm lỗi phần cứng bxCAN
    uint8_t tec = (CAN1->ESR >> 16) & 0xFF;
    uint8_t rec = (CAN1->ESR >> 24) & 0xFF;
    int16_t real_temp = (int16_t)tx_vehicle.data.coolant_temp - 40;
    char* mode_str[] = {"ECO", "NORMAL", "SPORT"};

    // 3. Xuất luồng Telemetry JSON kèm theo UDS VIN, Security, va Crash Freeze Frame
    printf("{\"rpm\":%u,\"spd\":%u,\"temp\":%d,\"thr\":%u,\"mil\":%u,\"brk\":%u,\"mode\":\"%s\",\"dtc\":\"%s%04X\",\"tec\":%u,\"rec\":%u,\"vin\":\"%s\",\"sec\":\"%s\",\"crash\":%u,\"edr\":\"%s\"}\r\n",
           tx_vehicle.data.rpm,
           tx_vehicle.data.speed,
           real_temp,
           tx_vehicle.data.throttle,
           tx_vehicle.data.mil_lamp,
           tx_vehicle.data.brake_pedal,
           mode_str[tx_vehicle.data.drive_mode],
           (tx_vehicle.data.dtc_code == 0) ? "NONE" : "P",
           tx_vehicle.data.dtc_code,
           tec, rec,
           last_vin_received,
           last_security_status,
           has_saved_crash,
           last_crash_str);

    osDelay(200);
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
