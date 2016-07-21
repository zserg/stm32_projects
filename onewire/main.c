/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 17/07/2016 14:56:07
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ownet.h"
#include "temp10.h"
#include "findtype.h"

#define MAXDEVICES   20
#define ONEWIRE_P 2

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
#ifdef __GNUC__
      /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
          set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
       #else
       #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  printf("Hello, world!\n\r");

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
   uchar FamilySN[MAXDEVICES][8];
   float current_temp;
   int i = 0;
   int j = 0;
   int NumDevices = 0;
   SMALLINT didRead = 0;

   //use port number for 1-wire
   uchar portnum = ONEWIRE_P;

   //----------------------------------------
   // Introduction header
   printf("\r\nTemperature\r\n");

   // attempt to acquire the 1-Wire Net
   if (!owAcquire(portnum,NULL))
   {
      printf("Acquire failed\r\n");
      while(owHasErrors())
         printf("  - Error %d\r\n", owGetErrorNum());
   }

//   while(1)
//  {
      j = 0;
      // Find the device(s)
      NumDevices = FindDevices(portnum, FamilySN, 0x10, MAXDEVICES);
      if (NumDevices>0)
      {
         printf("\r\n");
         // read the temperature and print serial number and temperature
         for (i = NumDevices; i; i--)
         {
            printf("(%d) ", j++);
            DisplaySerialNum(FamilySN[i-1]);
            if(owHasPowerDelivery(portnum))
            {
               didRead = ReadTemperature(portnum, FamilySN[i-1],&current_temp);
            }
            else
            {
               didRead = WeakReadTemperature(portnum, FamilySN[i-1],&current_temp);
            }

            if (didRead)
            {
               printf(" %5.1f Celsius\r\n", current_temp);
            }
            else
            {
               printf("  Convert failed.  Device is");
               if(!owVerify(portnum, FALSE))
                  printf(" not");
               printf(" present.\r\n");
               while(owHasErrors())
                  printf("  - Error %d\r\n", owGetErrorNum());
            }

         }
      }
      else
         printf("No temperature devices found!\r\n");

      owRelease(portnum);
//   }

  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */

  while (1)
  {

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  __HAL_RCC_AFIO_CLK_ENABLE();

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
 PUTCHAR_PROTOTYPE
 {
   /* Place your implementation of fputc here */
   /* e.g. write a character to the USART1 and Loop until the end of transmission */
   HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
 }

// -------------------------------------------------------------------------------
// Read and print the serial number.
//
void DisplaySerialNum(uchar sn[8])
{
   int i;
   for (i = 7; i>=0; i--)
      printf("%02X", (int)sn[i]);
}

//----------------------------------------------------------------------
// Write an array of bytes to the COM port, verify that it was
// sent out.  Assume that baud rate has been set.
//
// 'portnum'  - number 0 to MAX_PORTNUM-1.  This number was provided to
//              OpenCOM to indicate the port number.
// 'outlen'   - number of bytes to write to COM port
// 'outbuf'   - pointer ot an array of bytes to write
//
// Returns:  TRUE(1)  - success
//           FALSE(0) - failure
//
int WriteCOM(int portnum, int outlen, uchar *outbuf)
{
   if(portnum==1)
    {
      return HAL_UART_Transmit(&huart1, (uint8_t *)outbuf, outlen, 0xFFFF) == HAL_OK;
    }
   else
    {
      return HAL_UART_Transmit(&huart2, (uint8_t *)outbuf, outlen, 0xFFFF) == HAL_OK;
    }
}

//----------------------------------------------------------------------
// Read an array of bytes to the COM port, verify that it was
// sent out.  Assume that baud rate has been set.
//
// 'portnum'  - number 0 to MAX_PORTNUM-1.  This number was provided to
//               OpenCOM to indicate the port number.
// 'inlen'     - number of bytes to read from COM port
// 'inbuf'     - pointer to a buffer to hold the incomming bytes
//
// Returns: number of characters read
//
int ReadCOM(int portnum, int inlen, uchar *inbuf)
{
   uint16_t status;
   status = 0;

   if(portnum==1)
    {
      if(HAL_UART_Receive(&huart1, (uint8_t *)inbuf, inlen, 0xFFFF) == HAL_OK)
        {
          status = inlen;
        }
    }
   else
    {
      if(HAL_UART_Receive(&huart2, (uint8_t *)inbuf, inlen, 0xFFFF) == HAL_OK)
        {
          status = inlen;
        }
    }
   return status;
}

void FlushCOM(int portnum)
 {
   ;
 }

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
