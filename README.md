#include "stm32f10x.h"
#include "spi_flash.h"
#include "sim868.h"

// Define the speed limit in RPM (example value)
#define SPEED_LIMIT 3000

void PWM_Init(void);
void ADC_Init(void);
uint16_t Read_ADC(void);
void UART_Init(void);
void Send_Speed_Data(uint16_t speed);
void Flash_Init(void);
void Flash_Write_Speed(uint16_t speed);

int main(void) {
    uint16_t speed;

    // Initialize peripherals
    PWM_Init();
    ADC_Init();
    UART_Init();
    Flash_Init();
    SIM868_Init();

    while (1) {
        // Read the current speed from the ADC
        speed = Read_ADC();

        // Send the speed data to the monitoring system
        Send_Speed_Data(speed);

        // Store the speed data in flash memory
        Flash_Write_Speed(speed);

        // Check if the speed exceeds the limit
        if (speed > SPEED_LIMIT) {
            // Reduce the PWM duty cycle to slow down the motor
            TIM3->CCR1 = SPEED_LIMIT;
        } else {
            // Set the PWM duty cycle to the current speed
            TIM3->CCR1 = speed;
        }
    }
}

void PWM_Init(void) {
    // Enable GPIOA and TIM3 clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // Configure PA6 as alternate function push-pull (TIM3_CH1)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure TIM3 for PWM mode
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 4095;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);

    TIM_Cmd(TIM3, ENABLE);
}

void ADC_Init(void) {
    // Enable ADC1 and GPIOA clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA0 as analog input (ADC1_IN0)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure ADC1
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // Configure ADC1 regular channel0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);

    // Enable ADC1 reset calibration register
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));

    // Start ADC1 calibration
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));

    // Start ADC1 Software Conversion
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

uint16_t Read_ADC(void) {
    // Wait until conversion completion
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

    // Get the conversion value
    return ADC_GetConversionValue(ADC1);
}

void UART_Init(void) {
    // Enable USART1 and GPIOA clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // Configure PA9 (USART1_TX) as alternate function push-pull
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART1
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    // Enable USART1
    USART_Cmd(USART1, ENABLE);
}

void Send_Speed_Data(uint16_t speed) {
    // Convert speed to string
    char buffer[10];
    sprintf(buffer, "%d\n", speed);

    // Send speed data over UART
    for (int i = 0; buffer[i] != '\0'; i++) {
        USART_SendData(USART1, buffer[i]);
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }

    // Send speed data to SIM868 for transmission
    SIM868_SendData(buffer);
}

void Flash_Init(void) {
    // Initialize SPI for flash memory
    SPI_FLASH_Init();
}

void Flash_Write_Speed(uint16_t speed) {
    // Write speed data to flash memory
    uint8_t data[2];
    data[0] = (speed >> 8) & 0xFF;
    data[1] = speed & 0xFF;
    SPI_FLASH_Write(data, 0x000000, 2);
}
