#include "porting.h"
#include "grbl.h"

#if SUKA_V3

uint8_t UDR0 = 0;
uint8_t STEP_PORT = 0;
uint8_t DIRECTION_PORT = 0;
uint8_t DIR_X = 0;
uint8_t DIR_Y = 0;
uint8_t SEQ_X = 0;
uint8_t SEQ_Y = 0;
uint8_t EEPROM_Buffer[PAGE_SIZE];
uint8_t LASER_State = 0;

// EEPROM /////////////////////////////////////////////////////////////////////


void eeprom_init()
{
#if USE_EEPROM    
    uint8_t *p = EEPROM_Buffer;
    uint16_t idx = 0;

    HAL_FLASH_Unlock();
    for (idx = 0; idx < PAGE_SIZE; idx++)
    {
        *p++ = (*(__IO uint8_t*)(EEPROM_ADDRESS + idx));
    }
    if (EEPROM_Buffer[0] != SETTINGS_VERSION)
    {
        p = EEPROM_Buffer;

        for (idx = 0; idx < PAGE_SIZE; idx++)
        {
            *p++ = 0xFF;
        }
    }
#endif 
}

void _eeprom_flush()
{
#if USE_EEPROM    
    extern void FLASH_PageErase(uint32_t PageAddress);
    uint16_t *p = (uint16_t *)EEPROM_Buffer;
    uint32_t addr = EEPROM_ADDRESS;
    uint16_t size = PAGE_SIZE;

    FLASH_PageErase(EEPROM_ADDRESS);
    while (size > 0)
    {
        if (*p != 0xffff)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, *p++);
        }
        else
        {
            p++;
        }
        if (*p != 0xffff)
        {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + 2, *p++);
        }
        else
        {
            p++;
        }
        size -= 4;
        addr += 4;
    }
#endif
}

uint8_t _eeprom_get_char(uint32_t addr)
{
    return EEPROM_Buffer[addr];    
}

void _eeprom_put_char(uint32_t addr, uint8_t val)
{
    EEPROM_Buffer[addr] = val;
}

// FAN ////////////////////////////////////////////////////////////////////////

uint8_t _coolant_get_state()
{
    return (HAL_GPIO_ReadPin(FAN_GPIO_Port, FAN_Pin) == GPIO_PIN_SET);
}

void _coolant_start()
{
    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
}

void _coolant_stop()
{
    HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
}

// UART ///////////////////////////////////////////////////////////////////////

void _serial_init()
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);    
}
    
void _serial_write(uint8_t data)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&data, 1, 500);
}

void _serial_callback()
{
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE) != RESET))
    {
        UDR0 = ((uint8_t)(huart1.Instance->DR & 0xff));
        ISR_SERIAL_RX();
    }
    __HAL_UART_CLEAR_PEFLAG(&huart1);
}

// Laser //////////////////////////////////////////////////////////////////////

void _spindle_init()
{
    __HAL_TIM_SET_PRESCALER(&htim3, F_CPU / 1000000 - 1);
    __HAL_TIM_SET_AUTORELOAD(&htim3, SPINDLE_PWM_MAX_VALUE - 1);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    HAL_TIM_Base_Start(&htim3);
}

void _spindle_start(uint8_t pwm_value)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm_value);
    if (pwm_value == SPINDLE_PWM_OFF_VALUE) 
    {
        _spindle_stop();
    } 
    else 
    {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        LASER_State = 1;
    }
}

void _spindle_stop()
{
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    LASER_State = 0;
}

// Stepper ////////////////////////////////////////////////////////////////////

void _stepper_init()
{
    __HAL_TIM_SET_PRESCALER(&htim2, STEP_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim2, 0);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_NVIC_DisableIRQ(TIM2_IRQn);

    __HAL_TIM_SET_PRESCALER(&htim4, STEP_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(&htim4, 0);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
}

void _stepper_step_enable(uint8_t step_pulse_time, uint16_t period)
{
    __HAL_TIM_SET_AUTORELOAD(&htim4, step_pulse_time - 1);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_FLAG_UPDATE);
    __HAL_TIM_SET_AUTORELOAD(&htim2, period-1);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);    
}

void _stepper_step_disable()
{
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
}

void _stepper_step_prescaler(uint16_t prescaler)
{
    __HAL_TIM_SET_PRESCALER(&htim2, prescaler);
}

void _stepper_step_period(uint16_t period)
{
    __HAL_TIM_SET_AUTORELOAD(&htim2, period-1);
}

void _stepper_reset_enable()
{
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void _stepper_reset_clear()
{
    __HAL_TIM_CLEAR_IT(&htim4, TIM_FLAG_UPDATE);
}

void _stepper_reset_disable()
{
    HAL_NVIC_DisableIRQ(TIM4_IRQn);    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2)
    {
        ISR_TIMER2_Handler();
    }
    else if (htim == &htim4)
    {
        ISR_TIMER4_Handler();
    }
}

void step_x(unsigned char val) 
{
    if (val==0)
    {
        HAL_GPIO_WritePin(X_PORT, INA1_X_Pin|INB1_X_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(X_PORT, INA2_X_Pin|INB2_X_Pin, GPIO_PIN_RESET);
        SEQ_X = (SEQ_X + ((DIR_X) ? 3 : 1)) % 4;
    }
    else
    {
        switch(SEQ_X)
        {
        case 0:
            HAL_GPIO_WritePin(X_PORT, INA1_X_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(X_PORT, INB1_X_Pin, GPIO_PIN_SET);
            break;
        case 1:
            HAL_GPIO_WritePin(X_PORT, INA2_X_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(X_PORT, INB2_X_Pin, GPIO_PIN_SET);
            break;
        case 2:
            HAL_GPIO_WritePin(X_PORT, INA1_X_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(X_PORT, INB1_X_Pin, GPIO_PIN_RESET);
            break;
        case 3:
            HAL_GPIO_WritePin(X_PORT, INA2_X_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(X_PORT, INB2_X_Pin, GPIO_PIN_RESET);
            break;
        }
    }
}

void step_y(unsigned char val)
{
    if (val==0)
    {
        HAL_GPIO_WritePin(Y_PORT, INA2_Y_Pin|INB2_Y_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(Y_PORT, INA1_Y_Pin|INB1_Y_Pin, GPIO_PIN_RESET);
        SEQ_Y = (SEQ_Y + ((DIR_Y) ? 1 : 3)) % 4;
    }
    else
    {
        switch(SEQ_Y)
        {
        case 0:
            HAL_GPIO_WritePin(Y_PORT, INA2_Y_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Y_PORT, INB2_Y_Pin, GPIO_PIN_RESET);
            break;
        case 1:
            HAL_GPIO_WritePin(Y_PORT, INA1_Y_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(Y_PORT, INB1_Y_Pin, GPIO_PIN_RESET);
            break;
        case 2:
            HAL_GPIO_WritePin(Y_PORT, INA2_Y_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Y_PORT, INB2_Y_Pin, GPIO_PIN_SET);
            break;
        case 3:
            HAL_GPIO_WritePin(Y_PORT, INA1_Y_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(Y_PORT, INB1_Y_Pin, GPIO_PIN_SET);
            break;
        }        
    }
}

void process_step()
{
    DIR_X = (DIRECTION_PORT & (1 << X_DIRECTION_BIT));
    DIR_Y = (DIRECTION_PORT & (1 << Y_DIRECTION_BIT));
    step_x(STEP_PORT & (1 << X_STEP_BIT));
    step_y(STEP_PORT & (1 << Y_STEP_BIT));
}

// Button /////////////////////////////////////////////////////////////////////

void process_button()
{
    static uint32_t left = 0, right = 0, up = 0, down = 0, ok = 0;

    left  = (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin) == GPIO_PIN_RESET) ?  left+1 : 0;
    right = (HAL_GPIO_ReadPin(BTN_RIGHT_GPIO_Port, BTN_RIGHT_Pin) == GPIO_PIN_RESET) ?  right+1 : 0;
    up    = (HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin) == GPIO_PIN_RESET) ?  up+1 : 0;
    down  = (HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin) == GPIO_PIN_RESET) ?  down+1 : 0;
    ok    = (HAL_GPIO_ReadPin(BTN_OK_GPIO_Port, BTN_OK_Pin) == GPIO_PIN_RESET) ?  ok+1 : 0;
    
    if (left == BTN_TIMEOUT_COUNT) 
    { 
        left = 0;
        gc_execute_line("G91X-1Y0F300");
    }
    if (right == BTN_TIMEOUT_COUNT)
    { 
        right = 0;
        gc_execute_line("G91X1Y0F300");
    }
    if (up == BTN_TIMEOUT_COUNT)
    { 
        up = 0;
        gc_execute_line("G91X0Y1F300");
    }
    if (down == BTN_TIMEOUT_COUNT) 
    { 
        down = 0;
        gc_execute_line("G91X0Y-1F300");
    }
    if (ok == BTN_TIMEOUT_COUNT)
    { 
        if (LASER_State == 0)
        {
            gc_execute_line("M8M3S1G92X0Y0");
        }
        else
        {
            gc_execute_line("M9M3S0G92X0Y0");
        }
    }
}

///////////////////////////////////////////////////////////////////////////////

#endif //SUKA_V3
