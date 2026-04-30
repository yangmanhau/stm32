/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Joystick + laser + RGB LED + LCD + UART motor commands
  *                   Motor control via ESP-01S over USART3
  *                   Commands: F=forward, L=left, R=right, S=stop
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include "lcd.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PTD */
typedef enum {
    WIFI_STATE_IDLE = 0,
    WIFI_STATE_SEND_AT,
    WIFI_STATE_WAIT_AT,
    WIFI_STATE_SEND_CWMODE,
    WIFI_STATE_WAIT_CWMODE,
    WIFI_STATE_SEND_CWJAP,
    WIFI_STATE_WAIT_CWJAP,
    WIFI_STATE_SEND_CIFSR,
    WIFI_STATE_WAIT_CIFSR,
    WIFI_STATE_DONE,
    WIFI_STATE_FAIL
} wifi_state_t;

typedef enum {
    LASER_IDLE = 0,
    LASER_ARMED,
    LASER_PRIMING,
    LASER_FIRING,
    LASER_COOLDOWN
} laser_state_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define WIFI_SSID           "ESP_2F0F28"
#define WIFI_PASS           "thereisnospoon"

#define LASER_PIN           GPIO_PIN_4
#define LASER_PORT          GPIOC
#define LASER_PRIME_MS      1000U
#define LASER_FIRE_MS       1000U
#define LASER_COOLDOWN_MS   3000U

#define K1_PIN              GPIO_PIN_13
#define K1_PORT             GPIOC

#define JOY_SW_PIN          GPIO_PIN_2
#define JOY_SW_PORT         GPIOC

#define RGB_R_PIN           GPIO_PIN_5
#define RGB_R_PORT          GPIOB
#define RGB_G_PIN           GPIO_PIN_0
#define RGB_G_PORT          GPIOB
#define RGB_B_PIN           GPIO_PIN_1
#define RGB_B_PORT          GPIOB

#define DS18B20_PIN         GPIO_PIN_8
#define DS18B20_PORT        GPIOC

#define JOY_CENTER          2048U
#define JOY_DEAD            600U
#define JOY_FWD_THRESH      1200U
#define MOTOR_CMD_INTERVAL  100U
/* USER CODE END PD */

ADC_HandleTypeDef  hadc1;
ADC_HandleTypeDef  hadc2;
I2C_HandleTypeDef  hi2c2;
UART_HandleTypeDef huart3;
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
GPIO_PinState last_k1_state  = GPIO_PIN_SET;
GPIO_PinState last_jsw_state = GPIO_PIN_SET;
uint32_t last_jsw_press_tick  = 0;
uint8_t  jsw_has_been_pressed = 0;

laser_state_t laser_state = LASER_IDLE;
uint32_t      laser_tick  = 0;
char          laser_line[32] = "Laser:ready";

char     motor_label[16] = "STOP";
char     last_motor_cmd  = 'S';
uint32_t motor_cmd_tick  = 0;

char    esp_rx[256];
uint8_t esp_rx_byte;
volatile uint16_t esp_rx_index = 0;
volatile uint8_t  esp_rx_done  = 0;

char esp_cmd_rx[8]   = "none";
char prev_esp_cmd[8] = "";

wifi_state_t wifi_state      = WIFI_STATE_IDLE;
uint32_t     wifi_state_tick = 0;
char wifi_line1[32] = "WiFi:idle";
char wifi_line2[32] = "IP:none";

int32_t  ds18b20_raw       = -2032;
uint32_t ds18b20_last_tick = 0;

char prev_wifi1[32]  = "";
char prev_wifi2[32]  = "";
char prev_x[24]      = "";
char prev_y[24]      = "";
char prev_btn[8]     = "";
char prev_last[24]   = "";
char prev_laser[24]  = "";
char prev_temp[24]   = "";
char prev_motor[16]  = "";
/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART3_UART_Init(void);

static uint32_t read_adc1(void);
static uint32_t read_adc2(void);
static void     RGB_Set(uint8_t r, uint8_t g, uint8_t b);
static void     Motor_SendCmd(char cmd);
static void     laser_on_press(void);
static void     laser_on_release(void);
static void     laser_update(void);
static void     RGB_Update_From_State(void);
static void     ESP_Send(const char *cmd);
static void     ESP_ClearBuffer(void);
static void     ESP_StartReceiveIT(void);
static void     WiFi_Start(void);
static void     WiFi_Task(void);
static void     Drive_Task(uint32_t x_raw, uint32_t y_raw);
static void     LCD_UpdateUI(uint32_t x_raw, uint32_t y_raw,
                             GPIO_PinState jsw_now, int32_t temp_raw);
static void     ds_pin_out(void);
static void     ds_pin_in(void);
static void     ds_delay_us(uint16_t us);
static uint8_t  ds_start(void);
static void     ds_write(uint8_t data);
static uint8_t  ds_read_byte(void);
static int32_t  DS18B20_ReadRaw(void);

/* USER CODE BEGIN 0 */
static uint32_t read_adc1(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint32_t v = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return v;
}

static uint32_t read_adc2(void)
{
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 10);
    uint32_t v = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    return v;
}

static void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    HAL_GPIO_WritePin(RGB_R_PORT, RGB_R_PIN, r ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_G_PORT, RGB_G_PIN, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, b ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Motor_SendCmd(char cmd)
{
    uint32_t now = HAL_GetTick();
    if (cmd == last_motor_cmd && (now - motor_cmd_tick) < MOTOR_CMD_INTERVAL)
        return;

    sentAT(cmd);
    last_motor_cmd = cmd;
    motor_cmd_tick = now;

    /* Update ESP display immediately on send */
    esp_cmd_rx[0] = cmd;
    esp_cmd_rx[1] = '\0';
}

static void laser_on_press(void)
{
    if (laser_state != LASER_IDLE) return;
    laser_state = LASER_ARMED;
    strcpy(laser_line, "ARMED");
}

static void laser_on_release(void)
{
    if (laser_state != LASER_ARMED) return;
    laser_state = LASER_PRIMING;
    laser_tick  = HAL_GetTick();
    strcpy(laser_line, "PRIMING");
}

static void laser_update(void)
{
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - laser_tick;

    switch (laser_state)
    {
    case LASER_IDLE:
        strcpy(laser_line, "Laser:ready");
        break;

    case LASER_ARMED:
        strcpy(laser_line, "ARMED");
        break;

    case LASER_PRIMING:
        if (elapsed >= LASER_PRIME_MS)
        {
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_SET);
            laser_state = LASER_FIRING;
            laser_tick  = now;
            strcpy(laser_line, "FIRING");
        }
        else
        {
            uint32_t rem = LASER_PRIME_MS - elapsed;
            snprintf(laser_line, sizeof(laser_line), "PRIME:%3lums", rem);
        }
        break;

    case LASER_FIRING:
        if (elapsed >= LASER_FIRE_MS)
        {
            HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
            laser_state = LASER_COOLDOWN;
            laser_tick  = now;
            strcpy(laser_line, "COOLDOWN");
        }
        else
        {
            uint32_t rem = LASER_FIRE_MS - elapsed;
            snprintf(laser_line, sizeof(laser_line), "FIRE:%3lums", rem);
        }
        break;

    case LASER_COOLDOWN:
        if (elapsed >= LASER_COOLDOWN_MS)
        {
            laser_state = LASER_IDLE;
            strcpy(laser_line, "Laser:ready");
        }
        else
        {
            uint32_t rem = LASER_COOLDOWN_MS - elapsed;
            snprintf(laser_line, sizeof(laser_line), "CD:%lu.%lus",
                     rem / 1000U, (rem % 1000U) / 100U);
        }
        break;

    default:
        laser_state = LASER_IDLE;
        break;
    }
}

static void RGB_Update_From_State(void)
{
    switch (laser_state)
    {
    case LASER_ARMED:    RGB_Set(0,0,1); break;
    case LASER_PRIMING:  RGB_Set(0,1,0); break;
    case LASER_FIRING:   RGB_Set(0,1,0); break;
    case LASER_COOLDOWN: RGB_Set(1,0,0); break;
    case LASER_IDLE:
    default:             RGB_Set(1,1,1); break;
    }
}

static void Drive_Task(uint32_t x_raw, uint32_t y_raw)
{
    char cmd;

    if (y_raw < JOY_FWD_THRESH)
    {
        cmd = 'F';
        strcpy(motor_label, "FWD");
    }
    else if (x_raw < (JOY_CENTER - JOY_DEAD))
    {
        cmd = 'L';
        strcpy(motor_label, "LEFT");
    }
    else if (x_raw > (JOY_CENTER + JOY_DEAD))
    {
        cmd = 'R';
        strcpy(motor_label, "RIGHT");
    }
    else
    {
        cmd = 'S';
        strcpy(motor_label, "STOP");
    }

    Motor_SendCmd(cmd);
}





static void ds_delay_us(uint16_t us)
{
    uint32_t n = (uint32_t)us * 18U;
    while (n--) { __NOP(); }
}

static void ds_pin_out(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin   = DS18B20_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

static void ds_pin_in(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = DS18B20_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

static uint8_t ds_start(void)
{
    uint8_t present;
    ds_pin_out();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    ds_delay_us(500);
    ds_pin_in();
    ds_delay_us(70);
    present = (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    ds_delay_us(430);
    return present;
}

static void ds_write(uint8_t data)
{
    uint8_t i;
    for (i = 0; i < 8U; i++)
    {
        ds_pin_out();
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
        ds_delay_us(2);
        if (data & 0x01U) ds_pin_in();
        ds_delay_us(60);
        ds_pin_in();
        ds_delay_us(2);
        data >>= 1U;
    }
}

static uint8_t ds_read_byte(void)
{
    uint8_t i, val = 0U;
    for (i = 0; i < 8U; i++)
    {
        ds_pin_out();
        HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
        ds_delay_us(2);
        ds_pin_in();
        ds_delay_us(10);
        if (HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == GPIO_PIN_SET)
            val |= (uint8_t)(1U << i);
        ds_delay_us(55);
    }
    return val;
}

static int32_t  DS18B20_ReadRaw(void)
{
    uint8_t lo, hi;
    int16_t raw;
    if (!ds_start()) return -2032;
    ds_write(0xCCU); ds_write(0x44U);
    HAL_Delay(750);
    if (!ds_start()) return -2032;
    ds_write(0xCCU); ds_write(0xBEU);
    lo = ds_read_byte();
    hi = ds_read_byte();
    raw = (int16_t)(((uint16_t)hi << 8) | lo);
    if (raw == 0x0550) return -2032;
    return (int32_t)raw;
}

/* ?? LCD UpdateUI ??all clear regions fixed to avoid leftover pixels ?? */
static void LCD_UpdateUI(uint32_t x_raw, uint32_t y_raw,
                         GPIO_PinState jsw_now, int32_t temp_raw)
{
    char x_str[24], y_str[24], btn_str[8], last_str[24];
    char laser_str[32], temp_str[24], esp_cmd_str[8];

    uint32_t xcV = (x_raw * 330U) / 4095U;
    uint32_t ycV = (y_raw * 330U) / 4095U;

    snprintf(x_str,    sizeof(x_str),    "%4lu %lu.%02luV", x_raw, xcV/100U, xcV%100U);
    snprintf(y_str,    sizeof(y_str),    "%4lu %lu.%02luV", y_raw, ycV/100U, ycV%100U);
    strcpy(btn_str, (jsw_now == GPIO_PIN_RESET) ? "ON " : "OFF");

    if (jsw_has_been_pressed)
    {
        uint32_t e = HAL_GetTick() - last_jsw_press_tick;
        snprintf(last_str, sizeof(last_str), "%lu.%02lus", e/1000U, (e%1000U)/10U);
    }
    else
        strcpy(last_str, "never");

    snprintf(laser_str,   sizeof(laser_str),   "%s", laser_line);
    snprintf(esp_cmd_str, sizeof(esp_cmd_str),  "%s", esp_cmd_rx);

    if (temp_raw <= -2032)
        strcpy(temp_str, "NO SENSOR");
    else
    {
        int32_t t = temp_raw, neg = 0;
        if (t < 0) { neg = 1; t = -t; }
        int32_t whole = t/16, frac = ((t%16)*100)/16;
        if (neg) snprintf(temp_str, sizeof(temp_str), "-%ld.%02ldC", whole, frac);
        else     snprintf(temp_str, sizeof(temp_str),  "%ld.%02ldC", whole, frac);
    }

/* x1 starts well before the value so no glyph edge is ever left behind */
#define LCD_IF_CHANGED(prev, cur, x1, y1, x2, y2) \
    if (strcmp((prev),(cur)) != 0) { \
        LCD_Clear((x1),(y1),(x2),(y2), WHITE); \
        LCD_DrawString((x1),(y1),(cur)); \
        strcpy((prev),(cur)); \
    }

    LCD_IF_CHANGED(prev_wifi1,   wifi_line1,   50,  40, 239,  64)
    LCD_IF_CHANGED(prev_wifi2,   wifi_line2,   30,  65, 239,  89)
    LCD_IF_CHANGED(prev_x,       x_str,        25, 105, 239, 129)
    LCD_IF_CHANGED(prev_y,       y_str,        25, 135, 239, 159)
    LCD_IF_CHANGED(prev_btn,     btn_str,      45, 165,  95, 189)
    LCD_IF_CHANGED(prev_last,    last_str,    130, 165, 239, 189)
    LCD_IF_CHANGED(prev_laser,   laser_str,    60, 195, 239, 219)
    LCD_IF_CHANGED(prev_temp,    temp_str,     60, 225, 239, 249)
    LCD_IF_CHANGED(prev_motor,   motor_label,  60, 253, 239, 277)
    LCD_IF_CHANGED(prev_esp_cmd, esp_cmd_str,  35, 283, 239, 307)

#undef LCD_IF_CHANGED
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        char c = (char)esp_rx_byte;

        if (esp_rx_index < sizeof(esp_rx)-1)
        {
            esp_rx[esp_rx_index++] = c;
            esp_rx[esp_rx_index]   = '\0';
        }

        esp_rx_done = 1;
        HAL_UART_Receive_IT(&huart3, &esp_rx_byte, 1);
    }
}

void sendAT(const char *cmd) {
    HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
    HAL_Delay(500);
}

void readResponse(void) {
    char buffer[128] = {0};
    HAL_UART_Receive(&huart3, (uint8_t*)buffer, sizeof(buffer)-1, 1000);
    if (strlen(buffer) > 0){
    	if (buffer == "Hello from ESP01s client!"){
    		snprintf(wifi_line1, sizeof(wifi_line1), "WiFi:connect");
    	}
    }
}

void WifiSetUp(void){
	// 1. Test AT
	     sendAT("AT");
	     //readResponse();


	     // 2. Set AP mode
	     sendAT("AT+CWMODE=2");
	    // readResponse();

	     // 3. Configure AP SSID/password
	     sendAT("AT+CWSAP=\"ESP8266_AP_01\",\"12345678\",5,3");
	     //readResponse();

	     // Check AP IP
	     sendAT("AT+CIFSR");
	    // readResponse();

	     // 4. Start TCP server
	     sendAT("AT+CIPMUX=1");
	     sendAT("AT+CIPSERVER=1,80");
	     readResponse();
}
/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FSMC_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_I2C2_Init();
    MX_USART3_UART_Init();

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADCEx_Calibration_Start(&hadc2);

    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    RGB_Set(1, 1, 1);

    LCD_INIT();
    LCD_Clear(0, 0, 240, 320, WHITE);

    /* Static labels ??drawn once, never cleared */
    LCD_DrawString( 70,  10, "Lai Man To");
    LCD_DrawString( 10,  40, "WiFi:");
    LCD_DrawString( 10,  65, "IP:");
    LCD_DrawString( 10, 105, "X:");
    LCD_DrawString( 10, 135, "Y:");
    LCD_DrawString( 10, 165, "Btn:");
    LCD_DrawString(100, 165, "Last:");
    LCD_DrawString( 10, 195, "Laser:");
    LCD_DrawString( 10, 225, "Temp:");
    LCD_DrawString( 10, 253, "Motor:");
    LCD_DrawString( 10, 283, "ESP:");

    /* Initial values */
    LCD_DrawString( 60, 225, "NO SENSOR");
    LCD_DrawString( 60, 253, "STOP");
    LCD_DrawString( 35, 283, "none");

    strcpy(prev_temp,    "NO SENSOR");
    strcpy(prev_motor,   "STOP");
    strcpy(prev_esp_cmd, "none");

    Motor_SendCmd('S');

    WifiSetUp();

    while (1)
    {
        uint32_t      x_raw   = read_adc1();
        uint32_t      y_raw   = read_adc2();
        GPIO_PinState k1_now  = HAL_GPIO_ReadPin(K1_PORT, K1_PIN);
        GPIO_PinState jsw_now = HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN);

        /* K1: 2s forward burst then stop */
        if ((last_k1_state == GPIO_PIN_SET) && (k1_now == GPIO_PIN_RESET))
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(K1_PORT, K1_PIN) == GPIO_PIN_RESET)
            {
                Motor_SendCmd('F');
                strcpy(motor_label, "K1:FWD");
                HAL_Delay(2000);
                Motor_SendCmd('S');
                strcpy(motor_label, "K1:STOP");
                last_motor_cmd = 'S';
            }
        }
        last_k1_state = k1_now;

        /* SW falling edge ??ARMED (blue) */
        if ((last_jsw_state == GPIO_PIN_SET) && (jsw_now == GPIO_PIN_RESET))
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN) == GPIO_PIN_RESET)
            {
                last_jsw_press_tick  = HAL_GetTick();
                jsw_has_been_pressed = 1U;
                laser_on_press();
            }
        }

        /* SW rising edge ??PRIMING (green) */
        if ((last_jsw_state == GPIO_PIN_RESET) && (jsw_now == GPIO_PIN_SET))
        {
            HAL_Delay(20);
            if (HAL_GPIO_ReadPin(JOY_SW_PORT, JOY_SW_PIN) == GPIO_PIN_SET)
                laser_on_release();
        }

        last_jsw_state = jsw_now;

        /* DS18B20 every 2s */
        if (HAL_GetTick() - ds18b20_last_tick >= 2000U)
        {
            ds18b20_raw       = DS18B20_ReadRaw();
            ds18b20_last_tick = HAL_GetTick();
        }

        laser_update();
        RGB_Update_From_State();


            Drive_Task(x_raw, y_raw);


        LCD_UpdateUI(x_raw, y_raw, jsw_now, ds18b20_raw);

        HAL_Delay(10);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef       o = {0};
    RCC_ClkInitTypeDef       c = {0};
    RCC_PeriphCLKInitTypeDef p = {0};

    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState       = RCC_HSE_ON;
    o.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    o.HSIState       = RCC_HSI_ON;
    o.PLL.PLLState   = RCC_PLL_ON;
    o.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    o.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();

    c.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV2;
    c.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_2) != HAL_OK) Error_Handler();

    p.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    p.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&p) != HAL_OK) Error_Handler();
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};
    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
    s.Channel      = ADC_CHANNEL_10;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &s) != HAL_OK) Error_Handler();
}

static void MX_ADC2_Init(void)
{
    ADC_ChannelConfTypeDef s = {0};
    hadc2.Instance                   = ADC2;
    hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode    = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion       = 1;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();
    s.Channel      = ADC_CHANNEL_11;
    s.Rank         = ADC_REGULAR_RANK_1;
    s.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc2, &s) != HAL_OK) Error_Handler();
}

static void MX_I2C2_Init(void)
{
    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 100000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,    GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,     GPIO_PIN_SET);
    HAL_GPIO_WritePin(LASER_PORT, LASER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_R_PORT, RGB_R_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_G_PORT, RGB_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_B_PORT, RGB_B_PIN, GPIO_PIN_SET);

    g.Pin = GPIO_PIN_12;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &g);

    g.Pin = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOE, &g);

    g.Pin = LASER_PIN;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LASER_PORT, &g);

    g.Pin = RGB_R_PIN | RGB_G_PIN | RGB_B_PIN;
    HAL_GPIO_Init(GPIOB, &g);

    g.Pin  = JOY_SW_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(JOY_SW_PORT, &g);

    g.Pin = K1_PIN;
    HAL_GPIO_Init(K1_PORT, &g);

    g.Pin  = DS18B20_PIN;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DS18B20_PORT, &g);
}

static void MX_FSMC_Init(void)
{
    FSMC_NORSRAM_TimingTypeDef t = {0};
    hsram1.Instance = FSMC_NORSRAM_DEVICE;
    hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
    hsram1.Init.NSBank             = FSMC_NORSRAM_BANK1;
    hsram1.Init.DataAddressMux     = FSMC_DATA_ADDRESS_MUX_DISABLE;
    hsram1.Init.MemoryType         = FSMC_MEMORY_TYPE_SRAM;
    hsram1.Init.MemoryDataWidth    = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
    hsram1.Init.BurstAccessMode    = FSMC_BURST_ACCESS_MODE_DISABLE;
    hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
    hsram1.Init.WrapMode           = FSMC_WRAP_MODE_DISABLE;
    hsram1.Init.WaitSignalActive   = FSMC_WAIT_TIMING_BEFORE_WS;
    hsram1.Init.WriteOperation     = FSMC_WRITE_OPERATION_ENABLE;
    hsram1.Init.WaitSignal         = FSMC_WAIT_SIGNAL_DISABLE;
    hsram1.Init.ExtendedMode       = FSMC_EXTENDED_MODE_DISABLE;
    hsram1.Init.AsynchronousWait   = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
    hsram1.Init.WriteBurst         = FSMC_WRITE_BURST_DISABLE;
    t.AddressSetupTime      = 15;
    t.AddressHoldTime       = 15;
    t.DataSetupTime         = 255;
    t.BusTurnAroundDuration = 15;
    t.CLKDivision           = 16;
    t.DataLatency           = 17;
    t.AccessMode            = FSMC_ACCESS_MODE_A;
    if (HAL_SRAM_Init(&hsram1, &t, NULL) != HAL_OK) Error_Handler();
    __HAL_AFIO_FSMCNADV_DISCONNECTED();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
