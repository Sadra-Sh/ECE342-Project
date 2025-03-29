int main(void)
{

	/* Reset of all peripherals */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();


  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  //ov7670_init();
//  HAL_Delay(100);
  //ov7670_capture(frame);


  // Add variables you need here.
  generate_checkerboard();

  int cmdx = 0;
  int cmdy = 0;
  while (1) {

//	  cmdx++;
//	  cmdy++;
//	  sprintf(msg, "%d,%d\n", cmdx, cmdy);
//	  HAL_UART_Transmit_DMA(&huart5, (uint8_t *)msg, strlen(msg));
//	  sprintf(msg, "Sent %d,%d\r\n", cmdx, cmdy);
//	  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)msg, strlen(msg));

	  HAL_StatusTypeDef status = HAL_UART_Receive(&huart5, (uint8_t *)msg, sizeof(msg) - 1, 100);

	      // Null-terminate the received string to safely process it
	      msg[sizeof(msg) - 1] = '\0';  // Ensuring the buffer is null-terminated

	      if (status == HAL_OK) {
	          // Parse the received string to an integer
	          int receivedValue = atoi(msg);  // Convert string to integer

	          // Now you can use the integer value (receivedValue)
	          // For example, transmit it over UART3:
	          sprintf(msg, "Received Integer: %d\r\n", receivedValue);
	          HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);

	      } else {
	          // If timeout occurs, send a timeout message
	          sprintf(msg, "UART Receive Timeout\r\n");
	          HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
	      }


//	while(snapshot != 1) {
//	  HAL_Delay(10);
//	}
//	snapshot = 0;
//	while (HAL_UART_GetState(&huart5) != HAL_UART_STATE_READY) {
//			HAL_Delay(5);
//	}
//	print_buf();
//
//	HAL_DCMI_Resume(&hdcmi);
  }
}

