# SC2104 Lab 1

This lab provides an in-depth exploration of the STM32 board and the STM32Cube IDE software. Key components and peripherals covered include:

- **Buzzer**: Implementing sound signals for feedback or notifications.
- **OLED Display**: Utilizing a small, low-power display for visual output.
- **Ultrasound Sensor**: Integrating ultrasound technology for distance measurement and sensor applications.

This hands-on experience will enhance your understanding of embedded systems development using STM32.

## Things to Take Note:
Uncomment the following for the buzzer:

```c
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12, GPIO_PIN_SET);
//HAL_Delay(1000);
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12, GPIO_PIN_SET);
//HAL_Delay(1000);

//HAL_UART_Transmit(&huart3, sbuf, sizeof(sbuf), HAL_MAX_DELAY);

/*HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_SET);
//HAL_Delay(1000);
//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8, GPIO_PIN_SET);
//HAL_Delay(1000);*/