#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>


#define REG_DEVID        0x00
#define REG_POWER_CTL    0x2D
#define REG_DATA_FORMAT  0x31
#define REG_DATAX0       0x32
#define ADXL_ID_OK       0xE5


#define MOTION_THRESHOLD  50
#define TILT_THRESHOLD    130
#define DEBOUNCE_MS       50


typedef enum {
    MODE_NORMAL   = 0,
    MODE_AUTONOMO = 1
} SystemMode_t;


SPI_HandleTypeDef  hspi1;
UART_HandleTypeDef huart1;

char    uart_buf[200];
uint8_t rx_byte;
char    cmd_buf[32];
int     cmd_idx      = 0;
bool    log_enabled  = false;
bool    show_raw     = false;

int16_t offset_x = 0, offset_y = 0, offset_z = 0;
int16_t prev_x   = 0, prev_y   = 0, prev_z   = 0;

SystemMode_t current_mode  = MODE_NORMAL;
uint32_t     last_btn_tick = 0;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

void    ADXL_Write(uint8_t reg, uint8_t val);
uint8_t ADXL_Read(uint8_t reg);
void    Get_Accel_Data(int16_t *x, int16_t *y, int16_t *z);
void    Show_ACC(void);
void    Show_ACC_RAW(void);
void    ADXL_Calibrate(void);
void    Check_Motion(void);
void    Run_Autonomous(void);
void    Check_Button(void);
void    LED_On(void);
void    LED_Off(void);
void    Process_Command(char *cmd);
void    UART_Print(const char *msg);


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();

    UART_Print("\r\n=== ADXL345: SISTEMA INTEGRADO ===\r\n");
    UART_Print("Comandos: ACC | ACC RAW | ACC LOG ON | ACC LOG OFF\r\n");
    UART_Print("Boton PA8: cambia entre MODO NORMAL y MODO AUTONOMO\r\n");
    UART_Print("Modo actual: NORMAL\r\n");

    if (ADXL_Read(REG_DEVID) != ADXL_ID_OK) {
        UART_Print("ERROR: Sensor no detectado.\r\n");
        while(1);
    }

    ADXL_Write(REG_DATA_FORMAT, 0x01);
    ADXL_Write(REG_POWER_CTL,   0x08);
    HAL_Delay(100);

    ADXL_Calibrate();

    while (1)
    {

        Check_Button();

        if (current_mode == MODE_NORMAL)
        {
            if (HAL_UART_Receive(&huart1, &rx_byte, 1, 0) == HAL_OK)
            {
                if (rx_byte == '\r' || rx_byte == '\n')
                {
                    cmd_buf[cmd_idx] = '\0';
                    if (cmd_idx > 0) {
                        UART_Print("\r\n");
                        Process_Command(cmd_buf);
                    }
                    cmd_idx = 0;
                }
                else if (rx_byte == 8 || rx_byte == 127)
                {
                    if (cmd_idx > 0) cmd_idx--;
                }
                else if (cmd_idx < 31)
                {
                    cmd_buf[cmd_idx++] = rx_byte;
                }
            }

            if (log_enabled && cmd_idx == 0)
            {
                if (show_raw) Show_ACC_RAW();
                else          Show_ACC();
                Check_Motion();
                HAL_Delay(250);
            }
        }


        else
        {
            Run_Autonomous();
            HAL_Delay(200);
        }
    }
}


void Check_Button(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_RESET)
    {
        uint32_t now = HAL_GetTick();
        if ((now - last_btn_tick) > DEBOUNCE_MS)
        {
            last_btn_tick = now;

            if (current_mode == MODE_NORMAL)
            {
                current_mode = MODE_AUTONOMO;
                log_enabled  = false;
                cmd_idx      = 0;
                LED_Off();
                UART_Print("\r\n>>> MODO AUTONOMO <<<\r\n");
                UART_Print("Inclina el sensor para encender el LED\r\n");
                UART_Print("Umbral: ~30 grados\r\n");
            }
            else
            {
                current_mode = MODE_NORMAL;
                LED_Off();
                UART_Print("\r\n>>> MODO NORMAL <<<\r\n");
                UART_Print("Comandos: ACC | ACC RAW | ACC LOG ON | ACC LOG OFF\r\n");
            }
        }
    }
}


void Run_Autonomous(void)
{
    static bool tilted_prev = false;

    int16_t x, y, z;
    Get_Accel_Data(&x, &y, &z);


    sprintf(uart_buf, "[AUTO] X=%d Y=%d Z=%d | %s\r\n",
            x, y, z,
            ((x > TILT_THRESHOLD || x < -TILT_THRESHOLD ||
              y > TILT_THRESHOLD || y < -TILT_THRESHOLD)
             ? "INCLINADO LED=ON" : "nivelado  LED=OFF"));
    UART_Print(uart_buf);

    bool tilted = (x > TILT_THRESHOLD || x < -TILT_THRESHOLD ||
                   y > TILT_THRESHOLD || y < -TILT_THRESHOLD);

    if (tilted)
    {
        LED_On();
        if (!tilted_prev)
            UART_Print("*** ALERTA: INCLINACION DETECTADA ***\r\n");
    }
    else
    {
        LED_Off();
        if (tilted_prev)
            UART_Print("--- Robot nivelado, LED apagado ---\r\n");
    }

    tilted_prev = tilted;
}


void Get_Accel_Data(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t buf[6];
    uint8_t addr = REG_DATAX0 | 0x80 | 0x40;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive (&hspi1, buf,   6, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    *x = ((int16_t)((buf[1] << 8) | buf[0])) - offset_x;
    *y = ((int16_t)((buf[3] << 8) | buf[2])) - offset_y;
    *z = ((int16_t)((buf[5] << 8) | buf[4])) - offset_z;
}

void Show_ACC(void)
{
    int16_t x, y, z;
    Get_Accel_Data(&x, &y, &z);
    int32_t x_mg = (x * 78) / 10;
    int32_t y_mg = (y * 78) / 10;
    int32_t z_mg = (z * 78) / 10;

    sprintf(uart_buf, "ACC: X=%ld.%03ldg, Y=%ld.%03ldg, Z=%ld.%03ldg\r\n",
            x_mg/1000, (x_mg<0?-x_mg:x_mg)%1000,
            y_mg/1000, (y_mg<0?-y_mg:y_mg)%1000,
            z_mg/1000, (z_mg<0?-z_mg:z_mg)%1000);
    UART_Print(uart_buf);
}

void Show_ACC_RAW(void)
{
    int16_t x, y, z;
    Get_Accel_Data(&x, &y, &z);
    sprintf(uart_buf, "ACC RAW: X=%d, Y=%d, Z=%d\r\n", x, y, z);
    UART_Print(uart_buf);
}

void ADXL_Calibrate(void)
{
    int32_t sx=0, sy=0, sz=0;
    UART_Print("Calibrando ceros... ");
    for(int i=0; i<32; i++){
        uint8_t buf[6];
        uint8_t addr = REG_DATAX0 | 0x80 | 0x40;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive (&hspi1, buf, 6, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        sx += (int16_t)((buf[1] << 8) | buf[0]);
        sy += (int16_t)((buf[3] << 8) | buf[2]);
        sz += (int16_t)((buf[5] << 8) | buf[4]);
        HAL_Delay(10);
    }
    offset_x = sx/32; offset_y = sy/32; offset_z = sz/32;
    UART_Print("OK\r\n");
}


void Check_Motion(void)
{
    int16_t x, y, z;
    Get_Accel_Data(&x, &y, &z);

    int16_t dx = x - prev_x;
    int16_t dy = y - prev_y;
    int16_t dz = z - prev_z;

    if (dx > MOTION_THRESHOLD  || dx < -MOTION_THRESHOLD ||
        dy > MOTION_THRESHOLD  || dy < -MOTION_THRESHOLD ||
        dz > MOTION_THRESHOLD  || dz < -MOTION_THRESHOLD)
        LED_On();
    else
        LED_Off();

    prev_x = x; prev_y = y; prev_z = z;
}


void Process_Command(char *cmd)
{
    if (strcmp(cmd, "ACC") == 0) {
        show_raw = false;
        Show_ACC();
    }
    else if (strcmp(cmd, "ACC RAW") == 0) {
        show_raw = true;
        Show_ACC_RAW();
    }
    else if (strcmp(cmd, "ACC LOG ON") == 0) {
        log_enabled = true;
        UART_Print("LOG ACTIVADO\r\n");
    }
    else if (strcmp(cmd, "ACC LOG OFF") == 0) {
        log_enabled = false;
        LED_Off();
        UART_Print("LOG DESACTIVADO\r\n");
    }
    else {
        UART_Print("Comando desconocido.\r\n");
    }
}


void LED_On(void)  { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); }
void LED_Off(void) { HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);   }


void ADXL_Write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t ADXL_Read(uint8_t reg) {
    uint8_t addr = reg | 0x80;
    uint8_t val = 0;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &val, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return val;
}

void UART_Print(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}


void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                 |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi1);
}

static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    HAL_UART_Init(&huart1);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};


    __HAL_RCC_GPIOA_CLK_ENABLE();


    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_4;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


    GPIO_InitStruct.Pin   = GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_6|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin  = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}
