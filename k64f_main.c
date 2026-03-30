#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_i2c.h"
#include "fsl_uart.h"
#include "fsl_clock.h"

#include "FreeRTOS.h"
#include "task.h"

#include "MK64F12.h"

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 1 — DEFINES
   ════════════════════════════════════════════════════════════════════════════ */

#define LDR_ADC_BASE        ADC0
#define LDR_ADC_CHANNEL     12U
#define LDR_ADC_GROUP       0U
#define LIGHT_THRESHOLD     600U

#define MIC_ADC_CHANNEL     13U
#define MIC_ADC_GROUP       0U

#define LED_RED_GPIO        GPIOB
#define LED_RED_PORT        PORTB
#define LED_RED_PIN         22U

#define LED_GREEN_GPIO      GPIOB
#define LED_GREEN_PORT      PORTB
#define LED_GREEN_PIN       21U

#define TRIGGER_GPIO        GPIOB
#define TRIGGER_PORT        PORTB
#define TRIGGER_PIN         9U

#define BTN_PORT            PORTB
#define BTN_GPIO            GPIOB
#define BTN_PIN             23U
#define BTN_CLOCK           kCLOCK_PortB
#define TRIPLE_PRESS_WINDOW_MS  600U

/* ── BUZZER PTD0 ─────────────────────────────────────────────────────────── */
#define BUZZER_GPIO         GPIOD
#define BUZZER_PORT         PORTD
#define BUZZER_PIN          0U
#define BUZZER_BEEP_MS      1000U

#define ESP_UART_BASE       UART3
#define ESP_UART_PORT       PORTC
#define ESP_UART_RX_PIN     16U
#define ESP_UART_TX_PIN     17U
#define ESP_UART_BAUDRATE   1000000U

#define FRAME_START_0           0xAAU
#define FRAME_START_1           0x55U
#define TRANSCRIPT_BUF_SIZE     220U
#define TRANSCRIPT_TIMEOUT_MS   20000U
#define CAMERA_CMD              "TAKE_PHOTO\n"

#define I2C_BAUDRATE        100000U

#define ADXL345_ADDR              0x53U
#define ADXL345_REG_DEVID         0x00U
#define ADXL345_REG_BW_RATE       0x2CU
#define ADXL345_REG_POWER_CTL     0x2DU
#define ADXL345_REG_DATA_FORMAT   0x31U
#define ADXL345_REG_DATAX0        0x32U
#define ACCEL_DATA_LEN            6U
#define MOTION_THRESHOLD          50

#define STABLE_MS_REQUIRED  1000U
#define LOOP_MS             500U

#define LCD_ADDR            0x27U
#define LCD_BACKLIGHT       0x08U
#define LCD_EN              0x04U
#define LCD_RS              0x01U
#define LCD_CLEAR           0x01U
#define LCD_ENTRY_MODE      0x06U
#define LCD_DISPLAY_ON      0x0CU
#define LCD_4BIT_2LINE      0x28U
#define LCD_SET_DDRAM       0x80U

#define VL53L0X_ADDR        0x29U
#define DISTANCE_MIN_CM     7
#define DISTANCE_MAX_CM     30

/* ── Braille LEDs — LED1 moved to PTB20 to free PTD0 for buzzer ─────────── */
#define LED1_GPIO  GPIOB
#define LED1_PORT  PORTB
#define LED1_PIN   20U   /* was PTD0 — moved to PTB20 */

#define LED2_GPIO  GPIOC
#define LED2_PORT  PORTC
#define LED2_PIN   4U

#define LED3_GPIO  GPIOC
#define LED3_PORT  PORTC
#define LED3_PIN   12U

#define LED4_GPIO  GPIOC
#define LED4_PORT  PORTC
#define LED4_PIN   3U

#define LED5_GPIO  GPIOC
#define LED5_PORT  PORTC
#define LED5_PIN   2U

#define LED6_GPIO  GPIOA
#define LED6_PORT  PORTA
#define LED6_PIN   2U

#define BRAILLE_CHAR_DELAY_MS  800U

#define DECIMATION_FACTOR   2U
#define MAX_SAMPLES         19000U

#define TASK_STACK_SIZE     512U
#define TASK_PRIORITY       (tskIDLE_PRIORITY + 2U)

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 2 — SAMPLE BUFFER
   ════════════════════════════════════════════════════════════════════════════ */

static int16_t sampleBuffer[MAX_SAMPLES];

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 3 — ADC
   ════════════════════════════════════════════════════════════════════════════ */

static void ADC_Init(void)
{
    adc16_config_t adcConfig;
    ADC16_GetDefaultConfig(&adcConfig);
    adcConfig.resolution = kADC16_ResolutionSE12Bit;
    ADC16_Init(LDR_ADC_BASE, &adcConfig);
    ADC16_DoAutoCalibration(LDR_ADC_BASE);
    ADC16_EnableHardwareTrigger(LDR_ADC_BASE, false);
}

static uint16_t ADC_ReadChannel(uint32_t channel)
{
    adc16_channel_config_t ch;
    ch.channelNumber                        = channel;
    ch.enableInterruptOnConversionCompleted = false;
    ch.enableDifferentialConversion         = false;
    ADC16_SetChannelConfig(LDR_ADC_BASE, LDR_ADC_GROUP, &ch);
    while (!(ADC16_GetChannelStatusFlags(LDR_ADC_BASE, LDR_ADC_GROUP)
             & kADC16_ChannelConversionDoneFlag)) {}
    return (uint16_t)ADC16_GetChannelConversionValue(LDR_ADC_BASE, LDR_ADC_GROUP);
}

static inline uint32_t ADC_ReadLDR(void) { return ADC_ReadChannel(LDR_ADC_CHANNEL); }
static inline uint16_t ADC_ReadMic(void) { return ADC_ReadChannel(MIC_ADC_CHANNEL); }

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 4 — LEDs + TRIGGER
   ════════════════════════════════════════════════════════════════════════════ */

static void LED_Init(void)
{
    gpio_pin_config_t ledConfig = { kGPIO_DigitalOutput, 1U };
    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(LED_RED_PORT,   LED_RED_PIN,   kPORT_MuxAsGpio);
    GPIO_PinInit(LED_RED_GPIO,   LED_RED_PIN,   &ledConfig);
    PORT_SetPinMux(LED_GREEN_PORT, LED_GREEN_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(LED_GREEN_GPIO, LED_GREEN_PIN, &ledConfig);
}

static void LED_Red_ON(void)    { GPIO_PinWrite(LED_RED_GPIO,   LED_RED_PIN,   0U); }
static void LED_Red_OFF(void)   { GPIO_PinWrite(LED_RED_GPIO,   LED_RED_PIN,   1U); }
static void LED_Green_ON(void)  { GPIO_PinWrite(LED_GREEN_GPIO, LED_GREEN_PIN, 0U); }
static void LED_Green_OFF(void) { GPIO_PinWrite(LED_GREEN_GPIO, LED_GREEN_PIN, 1U); }

static void Trigger_Init(void)
{
    gpio_pin_config_t trigConfig = { kGPIO_DigitalOutput, 0U };
    PORT_SetPinMux(TRIGGER_PORT, TRIGGER_PIN, kPORT_MuxAsGpio);
    GPIO_PinInit(TRIGGER_GPIO, TRIGGER_PIN, &trigConfig);
}

static void Trigger_Allow(void) { GPIO_PinWrite(TRIGGER_GPIO, TRIGGER_PIN, 1U); }
static void Trigger_Block(void) { GPIO_PinWrite(TRIGGER_GPIO, TRIGGER_PIN, 0U); }

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 5 — BUZZER (PTD0 → 1kΩ → NPN Base → Collector to Buzzer-)
   ════════════════════════════════════════════════════════════════════════════ */

static void Buzzer_Init(void)
{
    CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinMux(BUZZER_PORT, BUZZER_PIN, kPORT_MuxAsGpio);
    gpio_pin_config_t buzzCfg = { kGPIO_DigitalOutput, 0U };
    GPIO_PinInit(BUZZER_GPIO, BUZZER_PIN, &buzzCfg);
    PRINTF("Buzzer Init Done (PTD0)\r\n");
}

static void Buzzer_ON(void)  { GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 1U); }
static void Buzzer_OFF(void) { GPIO_PinWrite(BUZZER_GPIO, BUZZER_PIN, 0U); }

static void Buzzer_Beep(uint32_t durationMs)
{
    Buzzer_ON();
    vTaskDelay(pdMS_TO_TICKS(durationMs));
    Buzzer_OFF();
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 6 — BUTTON (PTB23)
   ════════════════════════════════════════════════════════════════════════════ */

static void Button_Init(void)
{
    CLOCK_EnableClock(BTN_CLOCK);
    port_pin_config_t pinCfg   = {0};
    pinCfg.pullSelect          = kPORT_PullUp;
    pinCfg.passiveFilterEnable = kPORT_PassiveFilterEnable;
    pinCfg.mux                 = kPORT_MuxAsGpio;
    PORT_SetPinConfig(BTN_PORT, BTN_PIN, &pinCfg);
    gpio_pin_config_t gpioCfg = { kGPIO_DigitalInput, 0U };
    GPIO_PinInit(BTN_GPIO, BTN_PIN, &gpioCfg);
}

static inline bool Button_IsPressed(void)
{
    return GPIO_PinRead(BTN_GPIO, BTN_PIN) == 0U;
}

static uint8_t CountPresses(void)
{
    uint8_t    count     = 1;
    TickType_t windowEnd = xTaskGetTickCount()
                           + pdMS_TO_TICKS(TRIPLE_PRESS_WINDOW_MS);
    while (Button_IsPressed()) { vTaskDelay(pdMS_TO_TICKS(5)); }
    vTaskDelay(pdMS_TO_TICKS(30));
    while (xTaskGetTickCount() < windowEnd)
    {
        if (Button_IsPressed())
        {
            count++;
            while (Button_IsPressed()) { vTaskDelay(pdMS_TO_TICKS(5)); }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return count;
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 7 — UART3
   ════════════════════════════════════════════════════════════════════════════ */

static void UART3_Init(void)
{
    uart_config_t config;
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinMux(ESP_UART_PORT, ESP_UART_RX_PIN, kPORT_MuxAlt3);
    PORT_SetPinMux(ESP_UART_PORT, ESP_UART_TX_PIN, kPORT_MuxAlt3);
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = ESP_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;
    UART_Init(ESP_UART_BASE, &config, CLOCK_GetFreq(kCLOCK_BusClk));
}

static void UART3_SendString(const char *msg)
{
    UART_WriteBlocking(ESP_UART_BASE, (const uint8_t *)msg, strlen(msg));
}

static void SendCameraCommand(void)
{
    UART3_SendString(CAMERA_CMD);
    PRINTF("[TX] Camera command sent.\r\n");
}

/*
 * SendStabilityData()
 * Sends sensor gate status to ESP32 as a tagged CSV line.
 * Format: "STABILITY:STATUS,light,stable,dist,dist_cm\r\n"
 *
 * ESP32 detects the "STABILITY:" prefix, strips it, and POSTs
 * the CSV payload to Flask /stability.
 *
 * Called at the end of every RunSensorGate() — both PASS and BLOCKED.
 */
static void SendStabilityData(uint8_t lightOk, uint8_t stableOk,
                               uint8_t distOk,  uint16_t dist_cm)
{
    char msg[64];
    snprintf(msg, sizeof(msg),
             "STABILITY:%s,%u,%u,%u,%u\r\n",
             (lightOk && stableOk && distOk) ? "READY" : "BLOCKED",
             lightOk,
             stableOk,
             distOk,
             (unsigned int)dist_cm);
    UART3_SendString(msg);
    PRINTF("[TX] Stability: %s", msg);
}

static uint16_t ComputeChecksum(const int16_t *data, uint16_t count)
{
    uint16_t sum = 0;
    const uint8_t *bytes = (const uint8_t *)data;
    for (uint32_t i = 0; i < (uint32_t)count * 2U; i++)
        sum += bytes[i];
    return sum;
}

static void SendAudioFrame(const int16_t *data, uint16_t count)
{
    uint8_t  header[4];
    uint16_t checksum = ComputeChecksum(data, count);
    header[0] = FRAME_START_0; header[1] = FRAME_START_1;
    header[2] = (uint8_t)(count & 0xFFU);
    header[3] = (uint8_t)((count >> 8U) & 0xFFU);
    UART_WriteBlocking(ESP_UART_BASE, header, 4U);
    UART_WriteBlocking(ESP_UART_BASE, (const uint8_t *)data, (size_t)count * 2U);
    UART_WriteBlocking(ESP_UART_BASE, (const uint8_t *)&checksum, 2U);
    PRINTF("[TX] Sent %u samples, checksum=0x%04X\r\n", count, checksum);
}

static int UartDataReady(void)
{
    return (UART3->S1 & UART_S1_RDRF_MASK) ? 1 : 0;
}

/*
 * SendStabilityData()
 * Sends sensor gate status to ESP32 as a prefixed CSV line.
 * Format: "STABILITY:STATUS,light,stable,distOk,dist_cm\n"
 * Example: "STABILITY:READY,1,1,1,15\n"
 *
 * ESP32 detects "STABILITY:" prefix, strips it, then POSTs
 * the CSV payload to Flask /stability endpoint.
 */


/* ════════════════════════════════════════════════════════════════════════════
   SECTION 8 — I2C BUS
   ════════════════════════════════════════════════════════════════════════════ */

static void i2c_hw_init(void)
{
    CLOCK_EnableClock(kCLOCK_I2c0);
    CLOCK_EnableClock(kCLOCK_PortE);
    port_pin_config_t config = {
        kPORT_PullUp, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainEnable,
        kPORT_LowDriveStrength, kPORT_MuxAlt5, kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORTE, 24U, &config);
    PORT_SetPinConfig(PORTE, 25U, &config);
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
}

static void i2c_reinit(void)
{
    I2C_MasterDeinit(I2C0);
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 9 — I2C HELPERS
   ════════════════════════════════════════════════════════════════════════════ */

static void i2c_write_byte(uint8_t slave_addr, uint8_t data)
{
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = slave_addr; xfer.direction = kI2C_Write;
    xfer.data = &data; xfer.dataSize = 1U;
    xfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C0, &xfer);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void i2c_write_register(uint8_t slave_addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = slave_addr; xfer.direction = kI2C_Write;
    xfer.data = buf; xfer.dataSize = 2U;
    xfer.flags = kI2C_TransferDefaultFlag;
    status_t status = I2C_MasterTransferBlocking(I2C0, &xfer);
    if (status != kStatus_Success)
        PRINTF("I2C write error: %d\r\n", status);
}

static void i2c_read_registers(uint8_t slave_addr, uint8_t start_reg,
                                uint8_t *buffer, uint8_t length)
{
    i2c_master_transfer_t xfer;
    status_t status;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = slave_addr; xfer.direction = kI2C_Write;
    xfer.data = &start_reg; xfer.dataSize = 1U;
    xfer.flags = kI2C_TransferDefaultFlag;
    status = I2C_MasterTransferBlocking(I2C0, &xfer);
    if (status != kStatus_Success)
    {
        PRINTF("I2C read pointer error: %d\r\n", status);
        memset(buffer, 0, length);
        return;
    }
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = slave_addr; xfer.direction = kI2C_Read;
    xfer.data = buffer; xfer.dataSize = length;
    xfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C0, &xfer);
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 10 — VL53L0X HELPERS
   ════════════════════════════════════════════════════════════════════════════ */

static void vl53_write8(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = VL53L0X_ADDR; xfer.direction = kI2C_Write;
    xfer.data = buf; xfer.dataSize = 2U; xfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C0, &xfer);
}

static uint8_t vl53_read8(uint8_t reg)
{
    uint8_t val = 0;
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = VL53L0X_ADDR; xfer.direction = kI2C_Write;
    xfer.data = &reg; xfer.dataSize = 1U; xfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C0, &xfer);
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = VL53L0X_ADDR; xfer.direction = kI2C_Read;
    xfer.data = &val; xfer.dataSize = 1U; xfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C0, &xfer);
    return val;
}

static bool vl53_read_bytes(uint8_t reg, uint8_t *out, uint8_t len)
{
    i2c_master_transfer_t xfer;
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = VL53L0X_ADDR; xfer.direction = kI2C_Write;
    xfer.data = &reg; xfer.dataSize = 1U; xfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C0, &xfer);
    memset(&xfer, 0, sizeof(xfer));
    xfer.slaveAddress = VL53L0X_ADDR; xfer.direction = kI2C_Read;
    xfer.data = out; xfer.dataSize = len; xfer.flags = kI2C_TransferDefaultFlag;
    I2C_MasterTransferBlocking(I2C0, &xfer);
    return true;
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 11 — ADXL345
   ════════════════════════════════════════════════════════════════════════════ */

static void adxl345_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    uint8_t device_id = 0;
    i2c_read_registers(ADXL345_ADDR, ADXL345_REG_DEVID, &device_id, 1U);
    PRINTF("ADXL345 ID: 0x%02X (expected 0xE5)\r\n", device_id);
    i2c_write_register(ADXL345_ADDR, ADXL345_REG_BW_RATE,     0x0A);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_register(ADXL345_ADDR, ADXL345_REG_DATA_FORMAT, 0x00);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_register(ADXL345_ADDR, ADXL345_REG_POWER_CTL,   0x08);
    vTaskDelay(pdMS_TO_TICKS(10));
    PRINTF("ADXL345 Init Done\r\n");
}

static void adxl345_read_accel(int16_t *x, int16_t *y, int16_t *z)
{
    uint8_t raw[ACCEL_DATA_LEN];
    memset(raw, 0, sizeof(raw));
    i2c_read_registers(ADXL345_ADDR, ADXL345_REG_DATAX0, raw, ACCEL_DATA_LEN);
    *x = (int16_t)((raw[1] << 8) | raw[0]);
    *y = (int16_t)((raw[3] << 8) | raw[2]);
    *z = (int16_t)((raw[5] << 8) | raw[4]);
}

static int16_t abs_val(int16_t v) { return (v < 0) ? -v : v; }

static uint8_t is_moving(int16_t xn, int16_t yn, int16_t zn,
                          int16_t xp, int16_t yp, int16_t zp)
{
    return (abs_val(xn - xp) > MOTION_THRESHOLD ||
            abs_val(yn - yp) > MOTION_THRESHOLD ||
            abs_val(zn - zp) > MOTION_THRESHOLD) ? 1U : 0U;
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 12 — LCD
   ════════════════════════════════════════════════════════════════════════════ */

static void lcd_pulse_enable(uint8_t data)
{
    i2c_write_byte(LCD_ADDR, data | LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
    i2c_write_byte(LCD_ADDR, data & ~LCD_EN);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static void lcd_write4bits(uint8_t nibble)
{
    i2c_write_byte(LCD_ADDR, nibble | LCD_BACKLIGHT);
    lcd_pulse_enable(nibble | LCD_BACKLIGHT);
}

static void lcd_send(uint8_t value, uint8_t rs)
{
    uint8_t high = (value & 0xF0) | rs | LCD_BACKLIGHT;
    uint8_t low  = ((value << 4) & 0xF0) | rs | LCD_BACKLIGHT;
    lcd_write4bits(high);
    lcd_write4bits(low);
}

static void lcd_command(uint8_t cmd) { lcd_send(cmd, 0); vTaskDelay(pdMS_TO_TICKS(2)); }
static void lcd_char(uint8_t ch)     { lcd_send(ch, LCD_RS); }
static void lcd_print(const char *s) { while (*s) lcd_char((uint8_t)*s++); }

static void lcd_set_cursor(uint8_t row, uint8_t col)
{
    const uint8_t row_offsets[] = {0x00, 0x40};
    lcd_command(LCD_SET_DDRAM | (row_offsets[row] + col));
}

static void lcd_clear_display(void)
{
    lcd_command(LCD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static void lcd_print_row(uint8_t row, const char *str)
{
    char padded[17];
    snprintf(padded, sizeof(padded), "%-16s", str);
    lcd_set_cursor(row, 0);
    lcd_print(padded);
}

static void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_write4bits(0x30); vTaskDelay(pdMS_TO_TICKS(5));
    lcd_write4bits(0x30); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_write4bits(0x30); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_write4bits(0x20); vTaskDelay(pdMS_TO_TICKS(2));
    lcd_command(LCD_4BIT_2LINE);
    lcd_command(LCD_DISPLAY_ON);
    lcd_command(LCD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_command(LCD_ENTRY_MODE);
    lcd_set_cursor(0, 0); lcd_print("  Initializing  ");
    lcd_set_cursor(1, 0); lcd_print("  Please Wait   ");
    vTaskDelay(pdMS_TO_TICKS(2000));
    lcd_clear_display();
    PRINTF("LCD Init Done\r\n");
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 13 — VL53L0X
   ════════════════════════════════════════════════════════════════════════════ */

static bool vl53l0x_init(void)
{
    uint8_t id = vl53_read8(0xC0);
    PRINTF("VL53L0X ID: 0x%02X (expected 0xEE)\r\n", id);
    if (id != 0xEE) { PRINTF("VL53L0X NOT detected!\r\n"); return false; }

    vl53_write8(0x80, 0x01); vl53_write8(0xFF, 0x01); vl53_write8(0x00, 0x00);
    vl53_write8(0xFF, 0x00); vl53_write8(0x09, 0x00); vl53_write8(0x10, 0x00);
    vl53_write8(0x11, 0x00); vl53_write8(0x24, 0x01); vl53_write8(0x25, 0xFF);
    vl53_write8(0x75, 0x00); vl53_write8(0xFF, 0x01); vl53_write8(0x4E, 0x2C);
    vl53_write8(0x48, 0x00); vl53_write8(0x30, 0x20); vl53_write8(0xFF, 0x00);
    vl53_write8(0x30, 0x09); vl53_write8(0x54, 0x00); vl53_write8(0x31, 0x04);
    vl53_write8(0x32, 0x03); vl53_write8(0x40, 0x83); vl53_write8(0x46, 0x25);
    vl53_write8(0x60, 0x00); vl53_write8(0x27, 0x00); vl53_write8(0x50, 0x06);
    vl53_write8(0x51, 0x00); vl53_write8(0x52, 0x96); vl53_write8(0x56, 0x08);
    vl53_write8(0x57, 0x30); vl53_write8(0x61, 0x00); vl53_write8(0x62, 0x00);
    vl53_write8(0x64, 0x00); vl53_write8(0x65, 0x00); vl53_write8(0x66, 0xA0);
    vl53_write8(0xFF, 0x01); vl53_write8(0x22, 0x32); vl53_write8(0x47, 0x14);
    vl53_write8(0x49, 0xFF); vl53_write8(0x4A, 0x00); vl53_write8(0xFF, 0x00);
    vl53_write8(0x7A, 0x0A); vl53_write8(0x7B, 0x00); vl53_write8(0x78, 0x21);
    vl53_write8(0xFF, 0x01); vl53_write8(0x23, 0x34); vl53_write8(0x42, 0x00);
    vl53_write8(0x44, 0xFF); vl53_write8(0x45, 0x26); vl53_write8(0x46, 0x05);
    vl53_write8(0x40, 0x40); vl53_write8(0x0E, 0x06); vl53_write8(0x20, 0x1A);
    vl53_write8(0x43, 0x40); vl53_write8(0xFF, 0x00); vl53_write8(0x34, 0x03);
    vl53_write8(0x35, 0x44); vl53_write8(0xFF, 0x01); vl53_write8(0x31, 0x04);
    vl53_write8(0x4B, 0x09); vl53_write8(0x4C, 0x05); vl53_write8(0x4D, 0x04);
    vl53_write8(0xFF, 0x00); vl53_write8(0x44, 0x00); vl53_write8(0x45, 0x20);
    vl53_write8(0x47, 0x08); vl53_write8(0x48, 0x28); vl53_write8(0x67, 0x00);
    vl53_write8(0x70, 0x04); vl53_write8(0x71, 0x01); vl53_write8(0x72, 0xFE);
    vl53_write8(0x76, 0x00); vl53_write8(0x77, 0x00); vl53_write8(0xFF, 0x01);
    vl53_write8(0x0D, 0x01); vl53_write8(0xFF, 0x00); vl53_write8(0x80, 0x01);
    vl53_write8(0x01, 0xF8); vl53_write8(0xFF, 0x01); vl53_write8(0x8E, 0x01);
    vl53_write8(0x00, 0x01); vl53_write8(0xFF, 0x00); vl53_write8(0x80, 0x00);

    vTaskDelay(pdMS_TO_TICKS(10));
    vl53_write8(0x01, 0xFF);
    vl53_write8(0x0B, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    vl53_write8(0x00, 0x02);
    vTaskDelay(pdMS_TO_TICKS(100));
    PRINTF("VL53L0X Init Done\r\n");
    return true;
}

static uint16_t vl53l0x_read_mm(void)
{
    uint32_t timeout = 0;
    uint8_t  status  = 0;
    uint8_t  result[12] = {0};

    do {
        vTaskDelay(pdMS_TO_TICKS(5));
        status = vl53_read8(0x13);
        timeout++;
        if (timeout > 100U) { PRINTF("[VL53] Timeout\r\n"); return 0xFFFFU; }
    } while ((status & 0x01U) != 0U);

    vl53_read_bytes(0x14, result, 12);
    uint16_t mm = ((uint16_t)result[10] << 8) | result[11];
    vl53_write8(0x0B, 0x01);
    return mm;
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 14 — BRAILLE (LED1 now on PTB20)
   ════════════════════════════════════════════════════════════════════════════ */

typedef struct { char letter; uint8_t pattern; } BrailleChar;

static const BrailleChar braille_table[] = {
    { 'a', 0b000001 }, { 'b', 0b000011 }, { 'c', 0b001001 },
    { 'd', 0b011001 }, { 'e', 0b010001 }, { 'f', 0b001011 },
    { 'g', 0b011011 }, { 'h', 0b010011 }, { 'i', 0b001010 },
    { 'j', 0b011010 }, { 'k', 0b000101 }, { 'l', 0b000111 },
    { 'm', 0b001101 }, { 'n', 0b011101 }, { 'o', 0b010101 },
    { 'p', 0b001111 }, { 'q', 0b011111 }, { 'r', 0b010111 },
    { 's', 0b001110 }, { 't', 0b011110 }, { 'u', 0b100101 },
    { 'v', 0b100111 }, { 'w', 0b111010 }, { 'x', 0b101101 },
    { 'y', 0b111101 }, { 'z', 0b110101 },
    { ',', 0b000010 }, { ';', 0b000110 }, { ':', 0b010010 },
    { '.', 0b110010 }, { '?', 0b100110 }, { '!', 0b010110 },
    { '`', 0b000100 }, { '-', 0b100100 }, { ' ', 0b000000 },
};
#define BRAILLE_TABLE_SIZE (sizeof(braille_table) / sizeof(braille_table[0]))

static void braille_pins_init(void)
{
    CLOCK_EnableClock(kCLOCK_PortB);  /* LED1 on PTB20 — PortB already on */
    CLOCK_EnableClock(kCLOCK_PortD);  /* buzzer PTD0 — also in Buzzer_Init */
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortA);

    const port_pin_config_t outCfg = {
        kPORT_PullDisable, kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister
    };
    const gpio_pin_config_t gpioOut = { kGPIO_DigitalOutput, 0U };

    PORT_SetPinConfig(LED1_PORT, LED1_PIN, &outCfg); GPIO_PinInit(LED1_GPIO, LED1_PIN, &gpioOut);
    PORT_SetPinConfig(LED2_PORT, LED2_PIN, &outCfg); GPIO_PinInit(LED2_GPIO, LED2_PIN, &gpioOut);
    PORT_SetPinConfig(LED3_PORT, LED3_PIN, &outCfg); GPIO_PinInit(LED3_GPIO, LED3_PIN, &gpioOut);
    PORT_SetPinConfig(LED4_PORT, LED4_PIN, &outCfg); GPIO_PinInit(LED4_GPIO, LED4_PIN, &gpioOut);
    PORT_SetPinConfig(LED5_PORT, LED5_PIN, &outCfg); GPIO_PinInit(LED5_GPIO, LED5_PIN, &gpioOut);
    PORT_SetPinConfig(LED6_PORT, LED6_PIN, &outCfg); GPIO_PinInit(LED6_GPIO, LED6_PIN, &gpioOut);
}

static uint8_t braille_get_pattern(char c)
{
    for (int i = 0; i < (int)BRAILLE_TABLE_SIZE; i++)
        if (braille_table[i].letter == c)
            return braille_table[i].pattern;
    return 0x00;
}

static void braille_apply_pattern(uint8_t pattern)
{
    GPIO_PinWrite(LED1_GPIO, LED1_PIN, (pattern >> 0) & 0x01);
    GPIO_PinWrite(LED2_GPIO, LED2_PIN, (pattern >> 1) & 0x01);
    GPIO_PinWrite(LED3_GPIO, LED3_PIN, (pattern >> 2) & 0x01);
    GPIO_PinWrite(LED4_GPIO, LED4_PIN, (pattern >> 3) & 0x01);
    GPIO_PinWrite(LED5_GPIO, LED5_PIN, (pattern >> 4) & 0x01);
    GPIO_PinWrite(LED6_GPIO, LED6_PIN, (pattern >> 5) & 0x01);
}

static void braille_display_string(const char *str)
{
    while (*str)
    {
        braille_apply_pattern(braille_get_pattern(*str));
        vTaskDelay(pdMS_TO_TICKS(BRAILLE_CHAR_DELAY_MS));
        str++;
    }
    braille_apply_pattern(0x00);
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 15 — WAIT FOR TRANSCRIPT
   Milestone 5: checks for "BUZZER" → fires buzzer instead of Braille scroll
   ════════════════════════════════════════════════════════════════════════════ */

static int WaitForTranscript(void)
{
    char     buf[TRANSCRIPT_BUF_SIZE];
    uint16_t idx    = 0;
    uint8_t  rxByte = 0;

    TickType_t startTick    = xTaskGetTickCount();
    TickType_t timeoutTicks = pdMS_TO_TICKS(TRANSCRIPT_TIMEOUT_MS);
    memset(buf, 0, sizeof(buf));

    while ((xTaskGetTickCount() - startTick) < timeoutTicks)
    {
        if (UartDataReady())
        {
            UART_ReadBlocking(ESP_UART_BASE, &rxByte, 1);
            if (rxByte == '\r') continue;
            if (rxByte == '\n')
            {
                buf[idx] = '\0';

                if (strncmp(buf, "TRANSCRIPT:", 11) == 0)
                {
                    const char *text = buf + 11;

                    PRINTF("\r\n========================================\r\n");
                    PRINTF("Transcript: %s\r\n", text);
                    PRINTF("========================================\r\n\r\n");

                    /* ── Milestone 5: low confidence → BUZZER ── */
                    if (strcmp(text, "BUZZER") == 0)
                    {
                        PRINTF("[M5] Low confidence — firing buzzer!\r\n");
                        lcd_clear_display();
                        lcd_print_row(0, "Low Confidence!");
                        lcd_print_row(1, "OCR uncertain");
                        LED_Red_ON();
                        Buzzer_Beep(BUZZER_BEEP_MS);
                        LED_Red_OFF();
                        return 1;
                    }

                    /* ── High confidence → LCD + Braille ── */
                    lcd_clear_display();
                    lcd_print_row(0, text);
                    lcd_print_row(1, strlen(text) > 16 ? text + 16 : "");
                    PRINTF("Scrolling Braille...\r\n");
                    braille_display_string(text);
                    return 1;
                }

                idx = 0;
                memset(buf, 0, sizeof(buf));
                continue;
            }
            if (idx < (TRANSCRIPT_BUF_SIZE - 1U))
                buf[idx++] = (char)rxByte;
        }
        else { taskYIELD(); }
    }

    PRINTF("[RX] Timeout — no transcript received.\r\n");
    lcd_clear_display();
    lcd_print_row(0, "Timeout.");
    lcd_print_row(1, "Try again.");
    return -1;
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 16 — SENSOR GATE
   ════════════════════════════════════════════════════════════════════════════ */

static bool RunSensorGate(int16_t *x_prev, int16_t *y_prev, int16_t *z_prev,
                           uint32_t *stableMs, uint8_t *stableOk)
{
    uint32_t ldrValue = ADC_ReadLDR();
    uint8_t  lightOk  = (ldrValue >= LIGHT_THRESHOLD) ? 1U : 0U;

    int16_t x_now, y_now, z_now;
    adxl345_read_accel(&x_now, &y_now, &z_now);

    if (is_moving(x_now, y_now, z_now, *x_prev, *y_prev, *z_prev))
        { *stableMs = 0; *stableOk = 0U; }
    else
        { *stableMs += LOOP_MS; if (*stableMs >= STABLE_MS_REQUIRED) *stableOk = 1U; }
    *x_prev = x_now; *y_prev = y_now; *z_prev = z_now;

    uint16_t dist_mm = vl53l0x_read_mm();
    uint16_t dist_cm = (dist_mm != 0xFFFFU) ? (dist_mm / 10U) : 0U;
    uint8_t  distOk  = (dist_mm != 0xFFFFU &&
                        dist_cm >= (uint16_t)DISTANCE_MIN_CM &&
                        dist_cm <= (uint16_t)DISTANCE_MAX_CM) ? 1U : 0U;

    PRINTF("[LDR]   %4lu -> %s\r\n", ldrValue, lightOk ? "OK" : "TOO DARK");
    PRINTF("[ACCEL] -> %s (stableMs=%lu)\r\n", *stableOk ? "STABLE" : "MOVING", *stableMs);
    if (dist_mm == 0xFFFFU) PRINTF("[DIST]  SENSOR ERROR\r\n");
    else PRINTF("[DIST]  %u mm (%u cm) -> %s\r\n", dist_mm, dist_cm, distOk ? "OK" : "OUT OF RANGE");

    if (lightOk && *stableOk && distOk)
    {
        LED_Red_OFF(); LED_Green_ON(); Trigger_Allow();
        PRINTF("[GATE] ALL PASS\r\n");
        SendStabilityData(lightOk, *stableOk, distOk, dist_cm);
        return true;
    }

    LED_Red_ON(); LED_Green_OFF(); Trigger_Block();

    if      (!lightOk)           { lcd_print_row(0, "Low Light!");    lcd_print_row(1, "Need more light"); }
    else if (!(*stableOk))       { lcd_print_row(0, "Hold Steady!");  lcd_print_row(1, "Keep still 1s");  }
    else if (dist_mm == 0xFFFFU) { lcd_print_row(0, "Sensor Error!"); lcd_print_row(1, "Check VL53L0X");  }
    else if (dist_cm > (uint16_t)DISTANCE_MAX_CM) { lcd_print_row(0, "Move Closer!"); lcd_print_row(1, "Keep <30cm"); }
    else                         { lcd_print_row(0, "Too Close!");    lcd_print_row(1, "Back up >7cm");   }

    PRINTF("[GATE] BLOCKED - light=%u stable=%u dist=%u\r\n", lightOk, *stableOk, distOk);
    SendStabilityData(lightOk, *stableOk, distOk, dist_cm);
    return false;
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 17 — MAIN FREERTOS TASK
   ════════════════════════════════════════════════════════════════════════════ */

static void MainTask(void *pvParameters)
{
    (void)pvParameters;

    i2c_hw_init();
    adxl345_init();

    int16_t tx, ty, tz;
    adxl345_read_accel(&tx, &ty, &tz);
    PRINTF("ADXL test read: %d %d %d\r\n", tx, ty, tz);

    i2c_reinit();
    PRINTF("I2C Reinit Done\r\n");

    lcd_init();

    i2c_reinit();
    PRINTF("I2C Reinit After LCD Done\r\n");

    if (!vl53l0x_init())
    {
        lcd_print_row(0, "VL53L0X ERROR!");
        lcd_print_row(1, "Check wiring");
        while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }

    braille_pins_init();
    Buzzer_Init();

    /* ── Startup test: 3 short beeps ──
     * If you hear 3 beeps → buzzer wiring is correct.
     * If silent → check transistor base resistor and 5V supply.
     */
    PRINTF("Buzzer startup test (3 beeps)...\r\n");
    for (int i = 0; i < 3; i++)
    {
        Buzzer_Beep(150U);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    PRINTF("Buzzer test done.\r\n");

    int16_t  x_prev = 0, y_prev = 0, z_prev = 0;
    uint32_t stableMs = 0;
    uint8_t  stableOk = 0U;
    adxl345_read_accel(&x_prev, &y_prev, &z_prev);
    vTaskDelay(pdMS_TO_TICKS(500));

    bool systemOn = false;
    lcd_clear_display();
    lcd_print_row(0, "System OFF");
    lcd_print_row(1, "Press btn once");
    PRINTF("System OFF. Press button once to turn on.\r\n");

    while (1)
    {
        while (!Button_IsPressed()) { vTaskDelay(pdMS_TO_TICKS(10)); }
        uint8_t presses = CountPresses();
        PRINTF("[BTN] %u press(es) detected\r\n", presses);

        if (presses == 1)
        {
            systemOn = !systemOn;
            if (systemOn)
            {
                lcd_clear_display();
                lcd_print_row(0, "System ON");
                lcd_print_row(1, "2=Cam 3=Mic");
                PRINTF("System ON.\r\n");
                stableMs = 0; stableOk = 0U;
                adxl345_read_accel(&x_prev, &y_prev, &z_prev);
                LED_Red_OFF(); LED_Green_OFF(); Trigger_Block();
            }
            else
            {
                lcd_clear_display();
                lcd_print_row(0, "System OFF");
                lcd_print_row(1, "Press btn once");
                braille_apply_pattern(0x00);
                Buzzer_OFF();
                LED_Red_OFF(); LED_Green_OFF(); Trigger_Block();
                PRINTF("System OFF.\r\n");
            }
        }
        else if (presses == 2)
        {
            if (!systemOn) { PRINTF("[BTN] Ignored — system OFF.\r\n"); continue; }

            lcd_clear_display();
            lcd_print_row(0, "Checking...");
            lcd_print_row(1, "");

            bool gatePass = false;
            for (int attempt = 0; attempt < 8; attempt++)
            {
                gatePass = RunSensorGate(&x_prev, &y_prev, &z_prev, &stableMs, &stableOk);
                if (gatePass) break;
                vTaskDelay(pdMS_TO_TICKS(400));
            }

            if (!gatePass)
            {
                PRINTF("[CAM] Gate failed — photo aborted.\r\n");
                vTaskDelay(pdMS_TO_TICKS(2000));
                lcd_clear_display();
                lcd_print_row(0, "Photo aborted.");
                lcd_print_row(1, "Fix & try again");
                LED_Red_OFF();
                vTaskDelay(pdMS_TO_TICKS(2000));
                lcd_print_row(0, "System ON");
                lcd_print_row(1, "2=Cam 3=Mic");
                continue;
            }

            PRINTF("[CAM] Gate passed. Sending TAKE_PHOTO...\r\n");
            lcd_clear_display();
            lcd_print_row(0, "Taking photo...");
            lcd_print_row(1, "");

            SendCameraCommand();
            Trigger_Allow();
            vTaskDelay(pdMS_TO_TICKS(100));
            Trigger_Block();
            LED_Red_OFF(); LED_Green_OFF();

            lcd_print_row(0, "OCR running...");
            lcd_print_row(1, "");
            PRINTF("Waiting for OCR result...\r\n");

            WaitForTranscript();

            vTaskDelay(pdMS_TO_TICKS(3000));
            lcd_print_row(0, "System ON");
            lcd_print_row(1, "2=Cam 3=Mic");
            PRINTF("Ready.\r\n\r\n");
        }
        else if (presses >= 3)
        {
            if (!systemOn) { PRINTF("[BTN] Ignored — system OFF.\r\n"); continue; }

            lcd_clear_display();
            lcd_print_row(0, "Recording...");
            lcd_print_row(1, "");
            PRINTF("Recording %u samples...\r\n", MAX_SAMPLES);

            uint32_t   decimateCounter = 0;
            uint32_t   sampleIndex     = 0;
            TickType_t startTick       = xTaskGetTickCount();

            while (sampleIndex < MAX_SAMPLES)
            {
                uint16_t raw = ADC_ReadMic();
                decimateCounter++;
                if (decimateCounter >= DECIMATION_FACTOR)
                {
                    decimateCounter = 0;
                    sampleBuffer[sampleIndex++] = (int16_t)raw;
                }
            }

            TickType_t endTick    = xTaskGetTickCount();
            uint32_t   elapsedMs  = (endTick - startTick) * portTICK_PERIOD_MS;
            uint32_t   sampleRate = (MAX_SAMPLES * 1000U) / elapsedMs;
            PRINTF("Elapsed: %u ms  |  Sample rate: ~%u Hz\r\n", elapsedMs, sampleRate);

            lcd_print_row(0, "Processing...");
            lcd_print_row(1, "");

            PRINTF("Sending to ESP32...\r\n");
            SendAudioFrame(sampleBuffer, (uint16_t)MAX_SAMPLES);

            PRINTF("Waiting for transcript...\r\n");
            WaitForTranscript();

            vTaskDelay(pdMS_TO_TICKS(3000));
            lcd_print_row(0, "System ON");
            lcd_print_row(1, "2=Cam 3=Mic");
            PRINTF("Ready.\r\n\r\n");
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════════
   SECTION 18 — main()
   ════════════════════════════════════════════════════════════════════════════ */

int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();

    CLOCK_EnableClock(kCLOCK_PortB);
    PORT_SetPinMux(PORTB, 3U, kPORT_PinDisabledOrAnalog);

    ADC_Init();
    LED_Init();
    Trigger_Init();
    Button_Init();
    UART3_Init();

    PRINTF("Starting SEP600 combined system (M5)...\r\n");
    PRINTF("Buzzer: PTD0 -> 1kohm -> NPN Base\r\n");
    PRINTF("Braille LED1: PTB20 (moved from PTD0)\r\n");

    xTaskCreate(MainTask, "MainTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL);
    vTaskStartScheduler();

    while (1) {}
    return 0;
}
