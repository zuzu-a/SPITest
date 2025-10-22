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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spitest.h"
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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2007c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2007c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2007c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2007c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

DAC_HandleTypeDef hdac;

ETH_HandleTypeDef heth;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
// OEM719 GNSS receiver buffers and state
#define OEM_RX_BUF_SIZE 512
uint8_t oem_rx_buffer[OEM_RX_BUF_SIZE];
volatile uint16_t oem_rx_head = 0;
volatile bool oem_rx_byte_ready = false;
uint8_t oem_rx_single = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_DAC_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile bool slave_rx_done         = false;
volatile bool master_tx_done        = false;
volatile bool uart_busy             = false;
volatile bool button_request        = false;
volatile bool transfer_in_progress  = false;

uint32_t lastTick = 0;
uint8_t slave_rx_byte = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_13) { 
      uint32_t currentTick = HAL_GetTick();

      if ((currentTick - lastTick) > 250 && !transfer_in_progress) {
        button_request = true;
        lastTick = currentTick;
        HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
      }
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi5) {
        slave_rx_done = true;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi == &hspi4) {
        master_tx_done = true;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) uart_busy = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart6) {
        // Store received byte in circular buffer
        oem_rx_buffer[oem_rx_head] = oem_rx_single;
        oem_rx_head = (oem_rx_head + 1) % OEM_RX_BUF_SIZE;
        oem_rx_byte_ready = true;
        
        // Restart reception for next byte
        HAL_UART_Receive_IT(&huart6, &oem_rx_single, 1);
    }
}

const uint8_t data_buffer[4] = {'1', '3', '3', '7'};
      uint8_t slave_rx[4];


void SPILoop_HandleRequests()
{

  if (!button_request || transfer_in_progress) return;

  transfer_in_progress  = true;
  button_request        = false;
  slave_rx_done         = false;

  // Arm slave receive
  if (HAL_SPI_Receive_IT(&hspi5, (uint8_t *)slave_rx, 4) != HAL_OK) {
      char dbg[128];
      int n = snprintf(dbg, sizeof(dbg), "SLAVE_RX_IT failed: state=%d, err=0x%08lX\r\n",
                      (int)hspi5.State, (unsigned long)hspi5.ErrorCode);
      HAL_UART_Transmit(&huart3, (uint8_t*)dbg, (uint16_t)n, HAL_MAX_DELAY);
      transfer_in_progress = false;
      return;
  }


  // Master trnasmit
  if (HAL_SPI_Transmit_IT(&hspi4, (uint8_t *)data_buffer, 4) != HAL_OK) {
    HAL_UART_Transmit(&huart3, (uint8_t *)"MASTER_TX failed \r\n", sizeof("MASTER_TX failed \r\n"), HAL_MAX_DELAY);
    transfer_in_progress = false;
    return;
  }

  uint32_t t0 = HAL_GetTick();
  while(!slave_rx_done && (HAL_GetTick() - t0) < 2000) {}

  if (!slave_rx_done) {
    const char err[] = "SLAVE_RX timeout\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t*)err, sizeof(err)-1, HAL_MAX_DELAY);
    transfer_in_progress = false;
    return;
  }

  HAL_UART_Transmit(&huart3, (uint8_t*)"DATA RECEIVED AT SPI5: ", sizeof("DATA RECEIVED AT SPI5: "), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart3, slave_rx, sizeof(slave_rx), HAL_MAX_DELAY); // prints "1337" if OK
  HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);

  transfer_in_progress = false;
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
  MX_ETH_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  const char hello[] = "\r\n=== OEM719 GNSS Baudrate Scanner ===\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)hello, strlen(hello), HAL_MAX_DELAY);
  
  // Common OEM719 baudrates to test
  uint32_t baudrates[] = {115200, 230400, 460800, 57600, 38400, 19200, 9600};
  int num_baudrates = 7;
  bool found = false;
  uint32_t working_baudrate = 0;
  
  for (int i = 0; i < num_baudrates && !found; i++) {
      char msg[80];
      snprintf(msg, sizeof(msg), "\r\n[TEST %d/%d] Trying %lu baud...\r\n", 
               i+1, num_baudrates, (unsigned long)baudrates[i]);
      HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      
      // Reconfigure USART6 to test baudrate
      HAL_UART_Abort(&huart6);
      huart6.Init.BaudRate = baudrates[i];
      if (HAL_UART_Init(&huart6) != HAL_OK) {
          Error_Handler();
      }
      
      // Start interrupt-driven reception
      oem_rx_head = 0;  // Clear buffer
      HAL_UART_Receive_IT(&huart6, &oem_rx_single, 1);
      
      HAL_Delay(100);
      
      // Send UNLOG command (abbreviated)
      const char unlog[] = "UNLOG\r\n";
      HAL_UART_Transmit(&huart6, (uint8_t*)unlog, strlen(unlog), HAL_MAX_DELAY);
      
      HAL_Delay(500);  // Wait for response
      
      // Check if we got a response
      if (oem_rx_head > 5) {  // Should get at least "<OK" response
          char success[100];
          snprintf(success, sizeof(success), 
                   "*** SUCCESS! OEM719 responded at %lu baud! ***\r\n", 
                   (unsigned long)baudrates[i]);
          HAL_UART_Transmit(&huart3, (uint8_t*)success, strlen(success), HAL_MAX_DELAY);
          
          // Show what we received
          const char resp[] = "Response: ";
          HAL_UART_Transmit(&huart3, (uint8_t*)resp, strlen(resp), HAL_MAX_DELAY);
          HAL_UART_Transmit(&huart3, oem_rx_buffer, 
                           (oem_rx_head < 60) ? oem_rx_head : 60, HAL_MAX_DELAY);
          HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n\r\n", 3, HAL_MAX_DELAY);
          
          found = true;
          working_baudrate = baudrates[i];
  } else {
          const char fail[] = "No response.\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)fail, strlen(fail), HAL_MAX_DELAY);
      }
      
      HAL_UART_Abort(&huart6);
  }
  
  if (!found) {
      const char err1[] = "\r\n!!! ERROR: OEM719 not responding at ANY baudrate !!!\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)err1, strlen(err1), HAL_MAX_DELAY);
      const char err2[] = "Check:\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)err2, strlen(err2), HAL_MAX_DELAY);
      const char err3[] = "  1. Wiring: OEM719 TXD1 -> STM32 USART6_RX\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)err3, strlen(err3), HAL_MAX_DELAY);
      const char err4[] = "  2. Wiring: OEM719 RXD1 -> STM32 USART6_TX\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)err4, strlen(err4), HAL_MAX_DELAY);
      const char err5[] = "  3. Power: Is OEM719 powered and LED blinking?\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)err5, strlen(err5), HAL_MAX_DELAY);
      const char err6[] = "  4. Voltage: 3.3V logic levels (not 5V!)\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)err6, strlen(err6), HAL_MAX_DELAY);
      const char err7[] = "\r\nStopping here. Fix wiring and reset board.\r\n\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)err7, strlen(err7), HAL_MAX_DELAY);
      
      // Stop here - don't proceed
      while(1) {
          HAL_Delay(1000);
          HAL_GPIO_TogglePin(GPIOB, LD1_Pin);  // Blink LED to show error
      }
  }
  
  HAL_Delay(500);
  
  // ============================================================
  // Step 2: Now configure OEM719 at the working baudrate
  // ============================================================
  huart6.Init.BaudRate = working_baudrate;
  if (HAL_UART_Init(&huart6) != HAL_OK) {
      Error_Handler();
  }
  HAL_UART_Receive_IT(&huart6, &oem_rx_single, 1);
  
  oem_rx_head = 0;  // Clear buffer
  
  // Clear any pending logs
  const char unlog[] = "UNLOGALL\r\n";
  HAL_UART_Transmit(&huart6, (uint8_t*)unlog, strlen(unlog), HAL_MAX_DELAY);
  HAL_Delay(200);
  oem_rx_head = 0;
  
  // ============================================================
  // Step 3: Configure serial port at CURRENT baudrate
  // Critical: Set break detection OFF (stays at working baudrate)
  // ============================================================
  char serialcmd[80];
  snprintf(serialcmd, sizeof(serialcmd), "SERIALCONFIG COM1 %lu N 8 1 N OFF\r\n", 
           (unsigned long)working_baudrate);
  HAL_UART_Transmit(&huart6, (uint8_t*)serialcmd, strlen(serialcmd), HAL_MAX_DELAY);
  
  char msg_serial[100];
  snprintf(msg_serial, sizeof(msg_serial), "[STM32] SERIALCONFIG %lu baud, break OFF\r\n", 
           (unsigned long)working_baudrate);
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_serial, strlen(msg_serial), HAL_MAX_DELAY);
  
  HAL_Delay(500);
  
  // Check SERIALCONFIG response
  if (oem_rx_head > 0) {
      char resp[80];
      snprintf(resp, sizeof(resp), "[STM32] SERIALCONFIG response (%d bytes): ", oem_rx_head);
      HAL_UART_Transmit(&huart3, (uint8_t*)resp, strlen(resp), HAL_MAX_DELAY);
      
      for (int i = 0; i < oem_rx_head && i < 30; i++) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02X ", oem_rx_buffer[i]);
          HAL_UART_Transmit(&huart3, (uint8_t*)hex, strlen(hex), HAL_MAX_DELAY);
      }
      
      // Check for <OK (3C 4F 4B)
      bool got_ok = false;
      for (int i = 0; i < oem_rx_head - 2; i++) {
          if (oem_rx_buffer[i] == 0x3C && oem_rx_buffer[i+1] == 0x4F && oem_rx_buffer[i+2] == 0x4B) {
              got_ok = true;
              break;
          }
      }
      
      if (got_ok) {
          const char msg_ok[] = " [GOT <OK!]\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_ok, strlen(msg_ok), HAL_MAX_DELAY);
      } else {
          const char msg_bad[] = " [NO <OK - GARBLED!]\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_bad, strlen(msg_bad), HAL_MAX_DELAY);
      }
  }
  
  oem_rx_head = 0;
  HAL_Delay(500);
  
  const char msg_stay[] = "[STM32] Communication stable, ready for logging\r\n\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_stay, strlen(msg_stay), HAL_MAX_DELAY);
  
  // ============================================================
  // Step 4: Clear all existing logs
  // ============================================================
  const char unlogall[] = "UNLOGALL\r\n";
  HAL_UART_Transmit(&huart6, (uint8_t*)unlogall, strlen(unlogall), HAL_MAX_DELAY);
  
  const char msg_unlog[] = "[STM32] Cleared all logs\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_unlog, strlen(msg_unlog), HAL_MAX_DELAY);
  
  HAL_Delay(500);
  oem_rx_head = 0;
  
  // ============================================================
  // Step 5: CRITICAL DIAGNOSTIC - Check for binary format responses
  // ============================================================
  const char msg_diag[] = "[STM32] === DIAGNOSTICS ===\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_diag, strlen(msg_diag), HAL_MAX_DELAY);
  
  // Check if responses are BINARY (AA 44 12 1C = NovAtel binary sync)
  const char msg_check[] = "[STM32] Checking if OEM719 is in BINARY mode...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_check, strlen(msg_check), HAL_MAX_DELAY);
  
  // Request port statistics to see format
  const char portstat[] = "LOG PORTSTATSA ONCE\r\n";
  HAL_UART_Transmit(&huart6, (uint8_t*)portstat, strlen(portstat), HAL_MAX_DELAY);
  
  HAL_Delay(1000);
  
  if (oem_rx_head > 0) {
      // Check for binary header (AA 44 12 1C)
      bool is_binary = false;
      if (oem_rx_head >= 4) {
          if (oem_rx_buffer[0] == 0xAA && oem_rx_buffer[1] == 0x44 && 
              oem_rx_buffer[2] == 0x12 && oem_rx_buffer[3] == 0x1C) {
              is_binary = true;
          }
      }
      
      char resp[100];
      snprintf(resp, sizeof(resp), "[STM32] Response (%d bytes): ", oem_rx_head);
      HAL_UART_Transmit(&huart3, (uint8_t*)resp, strlen(resp), HAL_MAX_DELAY);
      
      for (int i = 0; i < oem_rx_head && i < 40; i++) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02X ", oem_rx_buffer[i]);
          HAL_UART_Transmit(&huart3, (uint8_t*)hex, strlen(hex), HAL_MAX_DELAY);
      }
      HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
      
      if (is_binary) {
          const char msg_bin[] = "[STM32] *** BINARY FORMAT DETECTED! ***\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_bin, strlen(msg_bin), HAL_MAX_DELAY);
          const char msg_fix[] = "[STM32] OEM719 is in BINARY mode, need to force ASCII!\r\n\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_fix, strlen(msg_fix), HAL_MAX_DELAY);
      } else {
          const char msg_unk[] = "[STM32] Format unclear - not standard binary\r\n\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_unk, strlen(msg_unk), HAL_MAX_DELAY);
      }
  }
  
  oem_rx_head = 0;
  
  // ============================================================
  // Step 6: FORCE ASCII mode explicitly
  // ============================================================
  const char msg_force[] = "[STM32] Forcing INTERFACEMODE to ASCII...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_force, strlen(msg_force), HAL_MAX_DELAY);
  
  const char ifmode[] = "INTERFACEMODE COM1 NOVATEL NOVATEL ON\r\n";
  HAL_UART_Transmit(&huart6, (uint8_t*)ifmode, strlen(ifmode), HAL_MAX_DELAY);
  
  HAL_Delay(500);
  
  if (oem_rx_head > 0) {
      char resp[80];
      snprintf(resp, sizeof(resp), "[STM32] INTERFACEMODE response (%d bytes): ", oem_rx_head);
      HAL_UART_Transmit(&huart3, (uint8_t*)resp, strlen(resp), HAL_MAX_DELAY);
      
      for (int i = 0; i < oem_rx_head && i < 30; i++) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02X ", oem_rx_buffer[i]);
          HAL_UART_Transmit(&huart3, (uint8_t*)hex, strlen(hex), HAL_MAX_DELAY);
      }
      
      // Check for ASCII <OK (3C 4F 4B)
      bool got_ok = false;
      for (int i = 0; i < oem_rx_head - 2; i++) {
          if (oem_rx_buffer[i] == 0x3C && oem_rx_buffer[i+1] == 0x4F && oem_rx_buffer[i+2] == 0x4B) {
              got_ok = true;
              break;
          }
      }
      
      if (got_ok) {
          const char msg_ok[] = " [GOT ASCII <OK!]\r\n\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_ok, strlen(msg_ok), HAL_MAX_DELAY);
      } else {
          const char msg_still[] = " [STILL GARBLED]\r\n\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_still, strlen(msg_still), HAL_MAX_DELAY);
      }
  }
  
  oem_rx_head = 0;
  
  // ============================================================
  // Step 7: Request TIMEA (ASCII format explicitly)
  // ============================================================
  const char timecmd[] = "LOG TIMEA ONTIME 1\r\n";
  HAL_UART_Transmit(&huart6, (uint8_t*)timecmd, strlen(timecmd), HAL_MAX_DELAY);
  
  const char msg_time[] = "[STM32] LOG TIMEA ONTIME 1\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_time, strlen(msg_time), HAL_MAX_DELAY);
  
  HAL_Delay(500);
  
  if (oem_rx_head > 0) {
      char resp[60];
      snprintf(resp, sizeof(resp), "[STM32] LOG response (%d bytes): ", oem_rx_head);
      HAL_UART_Transmit(&huart3, (uint8_t*)resp, strlen(resp), HAL_MAX_DELAY);
      
      for (int i = 0; i < oem_rx_head && i < 30; i++) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02X ", oem_rx_buffer[i]);
          HAL_UART_Transmit(&huart3, (uint8_t*)hex, strlen(hex), HAL_MAX_DELAY);
      }
      
      // Check for ASCII <OK (3C 4F 4B)
      bool got_ok = false;
      for (int i = 0; i < oem_rx_head - 2; i++) {
          if (oem_rx_buffer[i] == 0x3C && oem_rx_buffer[i+1] == 0x4F && oem_rx_buffer[i+2] == 0x4B) {
              got_ok = true;
              break;
          }
      }
      
      if (got_ok) {
          const char msg_ok[] = " [ASCII <OK!]\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_ok, strlen(msg_ok), HAL_MAX_DELAY);
      } else {
          const char msg_nok[] = "\r\n[STM32] WARNING: Response not clean ASCII!\r\n";
          HAL_UART_Transmit(&huart3, (uint8_t*)msg_nok, strlen(msg_nok), HAL_MAX_DELAY);
      }
  }
  
  oem_rx_head = 0;
  
  // ============================================================
  // Step 8: Wait 3 seconds for TIMEA periodic data
  // ============================================================
  const char msg_wait[] = "[STM32] Waiting 3 sec for TIMEA logs...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg_wait, strlen(msg_wait), HAL_MAX_DELAY);
  
  HAL_Delay(3000);
  
  if (oem_rx_head > 0) {
      char resp[80];
      snprintf(resp, sizeof(resp), "[STM32] *** SUCCESS! Got %d bytes of data! ***\r\n", oem_rx_head);
      HAL_UART_Transmit(&huart3, (uint8_t*)resp, strlen(resp), HAL_MAX_DELAY);
      
      const char msg_hex[] = "[STM32] First 100 bytes in hex:\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)msg_hex, strlen(msg_hex), HAL_MAX_DELAY);
      
      for (int i = 0; i < oem_rx_head && i < 100; i++) {
          char hex[4];
          snprintf(hex, sizeof(hex), "%02X ", oem_rx_buffer[i]);
          HAL_UART_Transmit(&huart3, (uint8_t*)hex, strlen(hex), HAL_MAX_DELAY);
          if ((i + 1) % 16 == 0) {
              HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
          }
      }
      HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n\r\n", 3, HAL_MAX_DELAY);
  } else {
      const char msg_fail[] = "[STM32] ERROR: No TIMEA data received!\r\n\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t*)msg_fail, strlen(msg_fail), HAL_MAX_DELAY);
  }
  
  oem_rx_head = 0;
  
  const char msg5[] = "=========================================\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg5, strlen(msg5), HAL_MAX_DELAY);
  const char msg6[] = "Now streaming ALL OEM719 data...\r\n\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t*)msg6, strlen(msg6), HAL_MAX_DELAY);
  
  // Start tail at current head to skip already-processed data
  uint16_t start_head = oem_rx_head;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint16_t oem_rx_tail = start_head;  // Start after already-shown data
  uint32_t last_forward_tick = HAL_GetTick();
  uint32_t last_debug_tick = HAL_GetTick();
  uint16_t last_rx_head = start_head;
  
  while (1)
  {
    SPILoop_HandleRequests();
    
    // Debug: Show data rate every 5 seconds
    uint32_t now = HAL_GetTick();
    if ((now - last_debug_tick) >= 5000) {
        last_debug_tick = now;
        
        uint16_t bytes_since_start = oem_rx_head - start_head;
        uint16_t bytes_this_period = oem_rx_head - last_rx_head;
        
        char dbg[100];
        snprintf(dbg, sizeof(dbg), "[DEBUG] Total: %d bytes | Last 5s: %d bytes\r\n", 
                 bytes_since_start, bytes_this_period);
        HAL_UART_Transmit(&huart3, (uint8_t*)dbg, strlen(dbg), HAL_MAX_DELAY);
        
        last_rx_head = oem_rx_head;
    }
    
    // Forward OEM719 data to PC via USART3 in real-time
    if (oem_rx_head != oem_rx_tail) {
        // Calculate how many bytes are available
        uint16_t bytes_avail;
        if (oem_rx_head >= oem_rx_tail) {
            bytes_avail = oem_rx_head - oem_rx_tail;
        } else {
            bytes_avail = OEM_RX_BUF_SIZE - oem_rx_tail;
        }
        
        if (bytes_avail > 0) {
            // Forward data to PC immediately
            HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, 
                                                         &oem_rx_buffer[oem_rx_tail], 
                                                         bytes_avail, 
                                                         100);
            if (status == HAL_OK) {
                oem_rx_tail = (oem_rx_tail + bytes_avail) % OEM_RX_BUF_SIZE;
            }
        }
    }
    
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_SLAVE;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_SLAVE;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 7;
  hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
