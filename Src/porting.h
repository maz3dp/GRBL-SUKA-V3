#ifndef __PORTING_H__
#define __PORTING_H__

#define SUKA_V3                 (1)

#if SUKA_V3

#pragma diag_suppress 1,111,177,550,1293,1295

#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"
#include "stm32f1xx_hal_tim.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define USE_EEPROM              (1)
#define F_CPU                   (SystemCoreClock)
#define PSTR(x)                 (x)
#define M_PI                    (3.1415926f)
#define trunc(x)                truncf(x)
#define round(x)                roundf(x)
#define _delay_ms(x)            HAL_Delay(x)
#define pgm_read_byte_near(x)   *(x)
#define PAGE_SIZE               (uint16_t)0x400
#define EEPROM_ADDRESS          ((uint32_t)0x0801fc00) 
#define BTN_TIMEOUT_COUNT       (10000)
#define sei()
typedef int                     bool;

// cpu_map.h
#define STEP_PRESCALER          (72)
#define X_PORT                  GPIOB
#define Y_PORT                  GPIOA
#define X_STEP_BIT              (0)
#define Y_STEP_BIT              (1)
#define Z_STEP_BIT              (2)
#define STEP_MASK               ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits
#define X_DIRECTION_BIT         (0)
#define Y_DIRECTION_BIT         (1)
#define Z_DIRECTION_BIT         (2)
#define DIRECTION_MASK          ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits
#define X_LIMIT_BIT             (0)
#define Y_LIMIT_BIT             (1)
#define Z_LIMIT_BIT             (2)
#define SPINDLE_PWM_FREQUENCY   (10000)   // KHz
#define SPINDLE_PWM_MAX_VALUE   (255)
#define SPINDLE_PWM_MIN_VALUE   (1)
#define SPINDLE_PWM_OFF_VALUE   (0)
#define SPINDLE_PWM_RANGE       (SPINDLE_PWM_MAX_VALUE-SPINDLE_PWM_MIN_VALUE)

// default.h
#define DEFAULT_X_STEPS_PER_MM          10.0f
#define DEFAULT_Y_STEPS_PER_MM          10.0f
#define DEFAULT_Z_STEPS_PER_MM          10.0f
#define DEFAULT_X_MAX_RATE              300.0f // mm/min
#define DEFAULT_Y_MAX_RATE              300.0f // mm/min
#define DEFAULT_Z_MAX_RATE              300.0f // mm/min
#define DEFAULT_X_ACCELERATION          (10.0f*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_Y_ACCELERATION          (10.0f*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_Z_ACCELERATION          (10.0f*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_X_MAX_TRAVEL            80.0f // mm NOTE: Must be a positive value.
#define DEFAULT_Y_MAX_TRAVEL            80.0f // mm NOTE: Must be a positive value.
#define DEFAULT_Z_MAX_TRAVEL            80.0f // mm NOTE: Must be a positive value.
#define DEFAULT_SPINDLE_RPM_MAX         255.0f // rpm
#define DEFAULT_SPINDLE_RPM_MIN         0.0f // rpm
#define DEFAULT_STEP_PULSE_MICROSECONDS 10
#define DEFAULT_STEPPING_INVERT_MASK    7
#define DEFAULT_DIRECTION_INVERT_MASK   7
#define DEFAULT_STEPPER_IDLE_LOCK_TIME  0 // msec (0-254, 255 keeps steppers enabled)
#define DEFAULT_STATUS_REPORT_MASK      1 // MPos enabled
#define DEFAULT_JUNCTION_DEVIATION      0.01f // mm
#define DEFAULT_ARC_TOLERANCE           0.002f // mm
#define DEFAULT_REPORT_INCHES           0 // false
#define DEFAULT_INVERT_ST_ENABLE        0 // false
#define DEFAULT_INVERT_LIMIT_PINS       0 // false
#define DEFAULT_SOFT_LIMIT_ENABLE       0 // false
#define DEFAULT_HARD_LIMIT_ENABLE       0  // false
#define DEFAULT_INVERT_PROBE_PIN        0 // false
#define DEFAULT_LASER_MODE              0 // false
#define DEFAULT_HOMING_ENABLE           0  // false
#define DEFAULT_HOMING_DIR_MASK         0 // move positive dir
#define DEFAULT_HOMING_FEED_RATE        25.0f // mm/min
#define DEFAULT_HOMING_SEEK_RATE        300.0f // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY   250 // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF          1.0f // mm

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart1;

extern uint8_t EEPROM_Buffer[0x400];
extern uint8_t STEP_PORT;
extern uint8_t DIRECTION_PORT;
extern uint8_t UDR0;

int grbl_main(void);
void ISR_SERIAL_RX(void);
void ISR_TIMER2_Handler(void);
void ISR_TIMER4_Handler(void);
void eeprom_init(void);
void _eeprom_flush();
uint8_t _eeprom_get_char(uint32_t addr);
void _eeprom_put_char(uint32_t addr, uint8_t val);
uint8_t _coolant_get_state(void);
void _coolant_start(void);
void _coolant_stop(void);
void _serial_init(void);
void _serial_write(uint8_t data);
void _serial_callback(void);
void _spindle_init(void);
void _spindle_start(uint8_t pwm_value);
void _spindle_stop(void);
void _stepper_init(void);
void _stepper_step_enable(uint8_t step_pulse_time, uint16_t period);
void _stepper_step_disable(void);
void _stepper_step_prescaler(uint16_t prescaler);
void _stepper_step_period(uint16_t period);
void _stepper_reset_enable(void);
void _stepper_reset_disable(void);
void _stepper_reset_clear(void);
void process_step(void);
void process_button(void);

#endif //SUKA_V3
#endif //__PORTING_H__

