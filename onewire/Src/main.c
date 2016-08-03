/**
  ******************************************************************************
  * File Name          : main.c
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
#define SMALL
#include "stdio.h"
#include "ownet.h"
#include "temp10.h"
#include "findtype.h"

#define MAXDEVICES   20
#define ONEWIRE_P 2
#define RXBUFFERSIZE 64

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
void DisplaySerialNum(uchar sn[8]);
int WeakReadTemperature(int, uchar*, float*);
/* Buffer used for reception */
 uint8_t aRxBuffer[RXBUFFERSIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);

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

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("Hello, world!\r\n");

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
      printf("FindDevices\r\n");
      // Find the device(s)
      NumDevices = FindDevices(portnum, FamilySN, 0x10, MAXDEVICES);
      printf("NumDevices=%d",NumDevices);
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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */


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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
   UART_HandleTypeDef *uart;

   printf("WriteCom %d\r\n",outlen);
   uart = (portnum == 0) ? &huart1 : &huart2;

   if(HAL_UART_Receive_IT(uart, (uint8_t *)aRxBuffer, RXBUFFERSIZE) != HAL_OK)
   {
      OWERROR(55);
      return 0;
   }

   return HAL_UART_Transmit(uart, (uint8_t *)outbuf, outlen, 0xFFFF) == HAL_OK;
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
   uint32_t timeout;

   UART_HandleTypeDef *uart;

   uart = (portnum == 0) ? &huart1 : &huart2;
   timeout = 0;

   printf("\r\nReadCom start (portnum=%d, inlen=%d)\r\n",portnum,inlen);

   while(--timeout)
   {
     if(uart->RxXferCount == inlen)
     {
        printf("\r\nReadCom status =%d (%x)\r\n",inlen,inbuf[0]);
        return inlen;
     }
   }
   printf("\r\nReadCom Timeout\r\n");
   return 0;
}

void FlushCOM(int portnum)
 {;}

int OpenCOM(int portnum, char *port_zstr)
{
  return 1;
}

int CloseCOM(int portnum)
{
  return 1;
}

int OpenCOMEx(char *port_zstr)
{
 switch(port_zstr[0]){
   case '0':
     return 0;
   case '1':
     return 1;
   case '2':
     return 2;
   default:
     return -1;
 }
}

void BreakCOM(int portnum)
{
   if(portnum==1)
    {
      HAL_LIN_SendBreak(&huart1);
    }
   else
    {
      HAL_LIN_SendBreak(&huart2);
    }
}

void SetBaudCOM(int portnum, uchar new_baud)
{
  ;
}

void msDelay(int delay)
{
  HAL_Delay(delay);
}

//----------------------------------------------------------------------
// Read the temperature of a DS1920/DS1820 without using Power Delivery
//
// 'portnum'     - number 0 to MAX_PORTNUM-1.  This number was provided to
//                 OpenCOM to indicate the port number.
// 'SerialNum'   - Serial Number of DS1920/DS1820 to read temperature from
// 'Temp '       - pointer to variable where that temperature will be
//                 returned
//
// Returns: TRUE(1)  temperature has been read and verified
//          FALSE(0) could not read the temperature, perhaps device is not
//                   in contact
//
int WeakReadTemperature(int portnum, uchar *SerialNum, float *Temp)
{
   uchar rt=FALSE;
   uchar send_block[30],lastcrc8;
   int send_cnt=0, tsht, i, loop=0;
   float tmp,cr,cpc;

   setcrc8(portnum,0);

   // set the device serial number to the counter device
   owSerialNum(portnum,SerialNum,FALSE);

   for (loop = 0; loop < 2; loop ++)
   {
      // access the device
      if (owAccess(portnum))
      {
         // send the convert temperature command
         owTouchByte(portnum,0x44);

         // sleep for 1 second
         msDelay(1000);

         // turn off the 1-Wire Net strong pull-up
         if (owLevel(portnum,MODE_NORMAL) != MODE_NORMAL)
            return FALSE;

         // access the device
         if (owAccess(portnum))
         {
            // create a block to send that reads the temperature
            // read scratchpad command
            send_block[send_cnt++] = 0xBE;
            // now add the read bytes for data bytes and crc8
            for (i = 0; i < 9; i++)
               send_block[send_cnt++] = 0xFF;

            // now send the block
            if (owBlock(portnum,FALSE,send_block,send_cnt))
            {
               // perform the CRC8 on the last 8 bytes of packet
               for (i = send_cnt - 9; i < send_cnt; i++)
                  lastcrc8 = docrc8(portnum,send_block[i]);

               // verify CRC8 is correct
               if (lastcrc8 == 0x00)
               {
                  // calculate the high-res temperature
                  tsht = send_block[1]/2;
                  if (send_block[2] & 0x01)
                     tsht |= -128;
                  tmp = (float)(tsht);
                  cr = send_block[7];
                  cpc = send_block[8];
                  if (((cpc - cr) == 1) && (loop == 0))
                     continue;
                  if (cpc == 0)
                     return FALSE;
                  else
                     tmp = tmp - (float)0.25 + (cpc - cr)/cpc;

                  *Temp = tmp;
                  // success
                  rt = TRUE;
               }
            }
         }
      }

   }

   // return the result flag rt
   return rt;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
