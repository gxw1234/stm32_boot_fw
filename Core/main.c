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
#include "cmsis_os.h"
#include "usb_device.h"
#include "init/uart_init.h"
#include "init/gpio_init.h"
#include "stm32h7xx_hal_qspi.h"
#include "init/gd25qxx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "tasks/lcd_task.h"
#include "tasks/ads1220_task.h"
#include "tasks/MP8865_task.h"
#include "tasks/test_iic_send.h"
#include "init/uart_init.h"
#include <stdarg.h>
#include <stdlib.h>
#include <stddef.h>
#include "tasks/debug_task.h"
#include "boot_mode.h"

/* USB命令处理任务声明 */
extern void usb_command_pc_to_st_task(void *pvParameters);

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


osThreadId defaultTaskHandle;
osThreadId printTaskHandle;  
TaskHandle_t lcdTaskHandle; 
TaskHandle_t ads1220TaskHandle;  
TaskHandle_t mp8865TaskHandle; 
TaskHandle_t UsbCmdTaskHandle; 
TaskHandle_t DebugTaskHandle; 
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void StartDefaultTask(void const * argument);
void StartPrintTask(void * argument);  /* 新的打印线程函数声明 */

/* USER CODE BEGIN PFP */
static void Jump_To_Application(void);
static void dump_hex(const uint8_t* addr, uint32_t base, size_t len);
static const uint8_t* find_app_end(const uint8_t* start, size_t max_len);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  MX_QUADSPI_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* GPIO初始化
  只有GPIO的配置 */
  MX_GPIO_Init();
  
  /* USER CODE BEGIN 2 */
  /* 初始化UART和printf重定向 */
  UART_Init();
  /* 初始化启动模式 */
  Boot_Mode_Init();


  // Boot_Mode_Set(BOOT_MODE_BOOTLOADER);
  // Boot_Mode_Set(BOOT_MODE_APPLICATION);


 /* 检查启动模式 */
 uint32_t boot_mode = Boot_Mode_Get();
 printf("Boot mode register: 0x%08lX\r\n", boot_mode);
 
 // 检查应用程序内存标记状态
 volatile uint32_t *app_status = (volatile uint32_t *)0x20010000;
 uint32_t status = *app_status;
 printf("Application memory status: 0x%08X\r\n", status);
 
 if (status == 0xDEADBEEF) {
     printf("Previous application execution detected\r\n");
 } else if (status == 0xCAFEBABE) {
     printf("Previous application UART config completed\r\n");
 } else if (status == 0x12345678) {
     printf("Previous application UART output completed\r\n");
 } else if (status == 0x00000000) {
     printf("No previous application execution detected\r\n");
 } else {
     printf("Unknown application status: 0x%08X\r\n", status);
 }
 
  if (boot_mode == BOOT_MODE_APPLICATION) {
    printf("Jumping to application...\r\n");

    if (GD25QXX_EnableXIP() != HAL_OK) {
      printf("ERROR: Failed to enable XIP mode\r\n");
    } else {
      /* 在跳转前打印XIP映射数据的头部与尾部用于一致性校验 */
      // const uint8_t* app = (const uint8_t*)0x90020000;
      // printf("=== XIP HEAD/TAIL DUMP ===\r\n");
      // printf("Head @ 0x%08lX:\r\n", (uint32_t)app);
      // dump_hex(app, (uint32_t)app, 64);

      // const size_t max_scan = 0x100000; /* 最多扫描1MB应用区域 */
      // const uint8_t* endp = find_app_end(app, max_scan);
      // const uint8_t* tail = (endp > app + 64) ? (endp - 64) : app;
      // printf("Tail around end @ 0x%08lX (end=0x%08lX):\r\n", (uint32_t)tail, (uint32_t)endp);
      // dump_hex(tail, (uint32_t)tail, 64);
      // printf("=== END DUMP ===\r\n");

      /* 打印结束后执行跳转 */
      Jump_To_Application();
      printf("Jump returned unexpectedly, staying in bootloader\r\n");
    }
  } else {
    printf("Staying in bootloader mode\r\n");
  }
  



  MX_USB_DEVICE_Init();  
  



  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /*打印线程 */
  // xTaskCreate(StartPrintTask, "PrintTask", configMINIMAL_STACK_SIZE * 2, NULL, 1, (TaskHandle_t*)&printTaskHandle);
  /*创建USB命令处理线程 */
  xTaskCreate(usb_command_pc_to_st_task, "UsbCmdTask", configMINIMAL_STACK_SIZE * 4, NULL, 7, &UsbCmdTaskHandle);  
  // /*创建调试线程 */
  // xTaskCreate(debug_task, "DebugTask", configMINIMAL_STACK_SIZE * 4, NULL, 5, &DebugTaskHandle);
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

      vTaskDelay(pdMS_TO_TICKS(1000));
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
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
}



/* GPIO initialization is now in Core/Src/init/gpio_init.c */

/* USER CODE BEGIN 4 */

static void dump_hex(const uint8_t* addr, uint32_t base, size_t len)
{
  for (size_t i = 0; i < len; i += 16) {
    printf("  %08lX: ", (unsigned long)(base + (uint32_t)i));
    for (size_t j = 0; j < 16 && i + j < len; ++j) {
      printf("%02X ", addr[i + j]);
    }
    printf("\r\n");
  }
}

static const uint8_t* find_app_end(const uint8_t* start, size_t max_len)
{
  if (max_len == 0) return start;
  const uint8_t* p = start + max_len;
  /* 向下对齐到4字节边界 */
  p = (const uint8_t*)((uint32_t)p & ~3u);
  while (p > start) {
    p -= 4;
    uint32_t w = *(volatile const uint32_t*)p;
    if (w != 0xFFFFFFFFu) {
      return p + 4; /* 返回第一个非全FF字的末尾 */
    }
  }
  return start; /* 全FF，认为没有有效数据 */
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  char msg[] = "Bootloader Version 1.0.0\n\r";  
  printf("%s", msg);
  /* Infinite loop */
  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPrintTask */
/**
  * @brief  Function implementing the printTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartPrintTask */
void StartPrintTask(void * argument)
{
  /* USER CODE BEGIN StartPrintTask */
  char msg[] = "Hello, this is print task!\r\n";
  uint32_t count = 0;
  char countMsg[50];
  
  /* Infinite loop */
  for(;;)
  {    
    sprintf(countMsg, "Count: %lu\r\n", count++);
    printf("%s", msg);
    printf("%s", countMsg);
    vTaskDelay(pdMS_TO_TICKS(1000)); // 使用FreeRTOS原生延时函数，1秒
  }
  /* USER CODE END StartPrintTask */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Configure QSPI XIP region for external Flash access
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x90000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16MB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;  // 允许执行，用于XIP
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  跳转到应用程序
  * @retval None
  */
static void Jump_To_Application(void)
{
  const uint32_t app_address = 0x90020000;   /* 固定应用地址 */
  const uint32_t stack_pointer = *(volatile uint32_t*)app_address;
  const uint32_t jump_address  = *(volatile uint32_t*)(app_address + 4);

  printf("Jumping to application at 0x%08lX...\r\n", app_address);
  /* 可选：简单打印用于最后一次确认 */
  printf("SP=0x%08lX, PC=0x%08lX\r\n", stack_pointer, jump_address);

  /* 清理运行环境（仅关中断，避免影响QSPI XIP运行） */
  __disable_irq();

  /* 设置向量表偏移和主堆栈指针 */
  SCB->VTOR = app_address;
  __DSB();
  __ISB();
  __set_MSP(stack_pointer);

  /* 跳转到复位向量 */
  void (*app_reset_handler)(void) = (void(*)(void))jump_address;
  app_reset_handler();

  /* 若返回，则视为失败 */
  printf("Jump returned unexpectedly.\r\n");
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
  printf("ERROR: Error_Handler\r\n");
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
