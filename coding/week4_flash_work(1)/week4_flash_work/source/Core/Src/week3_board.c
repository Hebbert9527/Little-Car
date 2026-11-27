#include "week3_board.h"

/* F446RE board support only: pin mux, clocks, DMA and NVIC. */
UART_HandleTypeDef huart1, huart3;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim5;
static DMA_HandleTypeDef dma1rx, dma3tx;
volatile uint32_t week3_init_errors;

static void uart_setup(UART_HandleTypeDef *h, USART_TypeDef *instance, uint32_t baud)
{
    h->Instance = instance;
    h->Init.BaudRate = baud;
    h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits = UART_STOPBITS_1;
    h->Init.Parity = UART_PARITY_NONE;
    h->Init.Mode = UART_MODE_TX_RX;
    h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(h) != HAL_OK)
        week3_init_errors++;
}

void HAL_UART_MspInit(UART_HandleTypeDef *h)
{
    GPIO_InitTypeDef g = {0};
    DMA_HandleTypeDef *d;
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    if (h->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();
        g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        g.Alternate = GPIO_AF7_USART1;
        d = &dma1rx;
        d->Instance = DMA2_Stream2;
        d->Init.Direction = DMA_PERIPH_TO_MEMORY;
        d->Init.Mode = DMA_CIRCULAR;
    }
    else if (h->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
        g.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        g.Alternate = GPIO_AF7_USART3;
        d = &dma3tx;
        d->Instance = DMA1_Stream3;
        d->Init.Direction = DMA_MEMORY_TO_PERIPH;
        d->Init.Mode = DMA_NORMAL;
    }
    else
        return;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(h->Instance == USART1 ? GPIOA : GPIOC, &g);
    d->Init.Channel = DMA_CHANNEL_4;
    d->Init.PeriphInc = DMA_PINC_DISABLE;
    d->Init.MemInc = DMA_MINC_ENABLE;
    d->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    d->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    d->Init.Priority = DMA_PRIORITY_HIGH;
    d->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(d) != HAL_OK)
        week3_init_errors++;
    if (h->Instance == USART1)
    {
        __HAL_LINKDMA(h, hdmarx, dma1rx);
        HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
        HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else
    {
        __HAL_LINKDMA(h, hdmatx, dma3tx);
        HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *h)
{
    if (h->Instance != I2C1)
        return;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}

void Week3Board_InitBluetooth(void)
{
    uart_setup(&huart3, USART3, 921600U);
}

void Week3Board_InitLidar(void)
{
    uart_setup(&huart1, USART1, 460800U);
}

HAL_StatusTypeDef Week3Board_InitI2c(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000U;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    return HAL_I2C_Init(&hi2c1);
}

void Week3Board_StartImuTimer(void)
{
    /* APB1 timer clock = 90 MHz: 90 MHz / 9000 / 100 = 100 Hz. */
    __HAL_RCC_TIM5_CLK_ENABLE();
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 8999U;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 99U;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
        week3_init_errors++;
    HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim5) != HAL_OK)
        week3_init_errors++;
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dma1rx);
}

void DMA1_Stream3_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&dma3tx);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void USART3_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart3);
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim5);
}
