#include "servo.h"
#include <string.h>
#include <stdlib.h>

#define SERVO_MIN_US 500.0f
#define SERVO_MAX_US 2500.0f
#define SERVO_PERIOD_US 20000.0f
#define SERVO_ANGLE_MAX 180.0f
#define SERVO_ANGLE_MIN 0.0f
#define UART_RX_LINE_LEN 16U
#define UART_RX_IDLE_TIMEOUT_MS 30U

extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

float now_angle = 90.0f;
float now_pwm_duty = 7.5f;

static uint8_t uart_rx_byte = 0;
static char uart_rx_line[UART_RX_LINE_LEN] = {0};
static volatile uint8_t uart_rx_len = 0;
static volatile uint8_t uart_rx_ready = 0;
static volatile uint32_t uart_last_rx_tick = 0;

static uint8_t Servo_IsDelimiter(uint8_t ch)
{
    return (ch == '\r') || (ch == '\n') || (ch == ',') || (ch == ';') ||
           (ch == ' ') || (ch == '\t');
}

void Servo_SetAngle(float angle)
{
    if(angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if(angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    now_angle = angle;
    float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / SERVO_ANGLE_MAX;
    uint16_t pwm_ccr = (uint16_t)(us + 0.5f);

    now_pwm_duty = ((float)pwm_ccr / SERVO_PERIOD_US) * 100.0f;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_ccr);
}

void VOFA_SendTelemetry(void)
{
    float data[2] = {now_angle, now_pwm_duty};
    uint8_t buf[sizeof(data) + 4U];
    const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

    memcpy(buf, data, sizeof(data));
    memcpy(&buf[sizeof(data)], tail, sizeof(tail));
    HAL_UART_Transmit(&huart1, buf, sizeof(buf), 20U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        if(uart_rx_ready == 0U)
        {
            if(Servo_IsDelimiter(uart_rx_byte))
            {
                if(uart_rx_len > 0U)
                {
                    uart_rx_line[uart_rx_len] = '\0';
                    uart_rx_ready = 1U;
                }
            }
            else if(uart_rx_len < (UART_RX_LINE_LEN - 1U))
            {
                uart_rx_line[uart_rx_len++] = (char)uart_rx_byte;
                uart_last_rx_tick = HAL_GetTick();
            }
            else
            {
                uart_rx_line[UART_RX_LINE_LEN - 1U] = '\0';
                uart_rx_ready = 1U;
            }
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
    }
}

void Servo_UART_Process(void)
{
    char line[UART_RX_LINE_LEN];
    uint8_t should_parse = 0U;

    __disable_irq();
    if(uart_rx_ready != 0U)
    {
        memcpy(line, uart_rx_line, UART_RX_LINE_LEN);
        uart_rx_len = 0U;
        uart_rx_ready = 0U;
        should_parse = 1U;
    }
    else if((uart_rx_len > 0U) &&
            ((HAL_GetTick() - uart_last_rx_tick) >= UART_RX_IDLE_TIMEOUT_MS))
    {
        uart_rx_line[uart_rx_len] = '\0';
        memcpy(line, uart_rx_line, UART_RX_LINE_LEN);
        uart_rx_len = 0U;
        should_parse = 1U;
    }
    __enable_irq();

    if(should_parse != 0U)
    {
        char *endptr;
        float target_angle = strtof(line, &endptr);

        if(endptr != line)
        {
            Servo_SetAngle(target_angle);
        }
    }
}

void Servo_UART_StartReceive(void)
{
    uart_rx_len = 0U;
    uart_rx_ready = 0U;
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1U);
}
