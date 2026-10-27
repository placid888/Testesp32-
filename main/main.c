#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// TinyUSB 核心與 HID/CDC API
#include "tinyusb.h"
#include "tinyusb_cdc_acm.h"
#include "class/hid/hid_device.h"

#define LCD_HOST       SPI2_HOST
#define LCD_PIN_CS     10
#define LCD_PIN_DC      8
#define LCD_PIN_MOSI   11
#define LCD_PIN_SCLK   12
#define LCD_PIN_RST     9
#define LCD_PIN_BL      7
#define BOOT_PIN        0

#define LCD_W         240
#define LCD_H         135
#define CMD_LINE_MAX  128  
#define HEARTBEAT_MS  1000

static const char *TAG = "macro_terminal";
static esp_lcd_panel_handle_t panel;
static QueueHandle_t line_queue;

typedef struct {
    bool connected;
    bool armed;
} dashboard_t;

static dashboard_t ui = {
    .connected = false,
    .armed = false, // 預設鎖定
};

static int64_t last_rx_us = 0;
static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;

// ==========================================
// [新增] USB HID 描述符與必要的回呼函式
// ==========================================
enum {
    REPORT_ID_KEYBOARD = 1,
    REPORT_ID_MOUSE = 2
};

// 告訴 PC 這是包含鍵盤與滑鼠的複合裝置
static uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(REPORT_ID_MOUSE))
};

// 回呼 1：PC 請求裝置描述符時觸發
uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance) {
    return desc_hid_report;
}

// 回呼 2：PC 讀取狀態時觸發 (此專案由 ESP32 主動發送，故回傳 0)
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    return 0;
}

// 回呼 3：PC 寫入狀態時觸發 (例如控制鍵盤 CapsLock 燈號，此處略過)
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
}
// ==========================================


// --- 簡易字元轉 HID 鍵碼 (支援英數字與空白) ---
#define CHAR_TO_HID(c) ((c) >= 'a' && (c) <= 'z' ? HID_KEY_A + ((c) - 'a') : \
                        (c) >= 'A' && (c) <= 'Z' ? HID_KEY_A + ((c) - 'A') : \
                        (c) >= '1' && (c) <= '9' ? HID_KEY_1 + ((c) - '1') : \
                        (c) == '0' ? HID_KEY_0 : \
                        (c) == ' ' ? HID_KEY_SPACE : 0)

// --- 執行真正的硬體動作 ---
static void execute_macro_command(const char *cmd) {
    char key, extra;
    int x, y, ms;
    uint8_t keycode[6] = {0};

    // [修正] 加入正確的 Report ID
    if (sscanf(cmd, "KD:%c%c", &key, &extra) == 1) {
        keycode[0] = CHAR_TO_HID(key);
        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
    } else if (sscanf(cmd, "KU:%c%c", &key, &extra) == 1) {
        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
    } else if (sscanf(cmd, "TAP:%c%c", &key, &extra) == 1) {
        keycode[0] = CHAR_TO_HID(key);
        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
        vTaskDelay(pdMS_TO_TICKS(30)); 
        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
    } else if (sscanf(cmd, "MM:%d,%d%c", &x, &y, &extra) == 2) {
        tud_hid_mouse_report(REPORT_ID_MOUSE, 0, (int8_t)x, (int8_t)y, 0, 0);
    } else if (strcmp(cmd, "MC:L") == 0) {
        tud_hid_mouse_report(REPORT_ID_MOUSE, MOUSE_BUTTON_LEFT, 0, 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(30));
        tud_hid_mouse_report(REPORT_ID_MOUSE, 0, 0, 0, 0, 0);
    } else if (sscanf(cmd, "DLY:%d%c", &ms, &extra) == 1 && ms >= 0) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else if (strcmp(cmd, "RELEASE_ALL") == 0) {
        tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
        tud_hid_mouse_report(REPORT_ID_MOUSE, 0, 0, 0, 0, 0);
    } else {
        ESP_LOGW(TAG, "無效指令: %s", cmd);
    }
}

// --- CDC 通訊接收回呼函數 ---
static void cdc_rx_cb(int itf, cdcacm_event_t *event) {
    uint8_t buf[64];
    size_t n = 0;
    if (tinyusb_cdcacm_read(itf, buf, sizeof(buf), &n) != ESP_OK) return;

    static char line[CMD_LINE_MAX];
    static size_t used = 0;

    portENTER_CRITICAL(&state_lock);
    ui.connected = true;
    last_rx_us = esp_timer_get_time();
    portEXIT_CRITICAL(&state_lock);

    for (size_t i = 0; i < n; ++i) {
        char c = (char)buf[i];
        if (c == '\r') continue;
        if (c == '|' || c == '\n') {
            if (used > 0) {
                line[used] = '\0';
                xQueueSend(line_queue, line, 0);
                used = 0;
            }
        } else if (used < sizeof(line) - 1) {
            line[used++] = c;
        }
    }
}

// --- 指令處理任務 (直譯器) ---
static void command_task(void *arg) {
    char cmd[CMD_LINE_MAX];
    while (true) {
        if (xQueueReceive(line_queue, cmd, portMAX_DELAY) == pdTRUE) {
            bool armed;
            portENTER_CRITICAL(&state_lock);
            armed = ui.armed;
            portEXIT_CRITICAL(&state_lock);

            if (armed || strcmp(cmd, "RELEASE_ALL") == 0) {
                execute_macro_command(cmd);
            } else {
                ESP_LOGW(TAG, "[LOCKED] 拒絕執行: %s", cmd);
            }
        }
    }
}

// --- 看門狗與安全任務 ---
static void safety_task(void *arg) {
    bool timed_out = false;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(50));
        int64_t now = esp_timer_get_time();
        bool expired = false;

        portENTER_CRITICAL(&state_lock);
        if (ui.armed && last_rx_us > 0 && (now - last_rx_us) > HEARTBEAT_MS * 1000LL) {
            ui.armed = false;
            expired = true;
        }
        portEXIT_CRITICAL(&state_lock);

        if (expired && !timed_out) {
            ESP_LOGW(TAG, "[看門狗] 逾時斷線！強制釋放按鍵！");
            execute_macro_command("RELEASE_ALL");
            timed_out = true;
        } else if (!expired) {
            timed_out = false;
        }
    }
}

// --- BOOT 按鍵中斷 (切換 ARMED / LOCKED) ---
static void IRAM_ATTR boot_isr(void *arg) {
    static TickType_t last_tick = 0;
    TickType_t now = xTaskGetTickCountFromISR();
    if (now - last_tick < pdMS_TO_TICKS(200)) return; 
    last_tick = now;

    portENTER_CRITICAL_ISR(&state_lock);
    ui.armed = !ui.armed;
    portEXIT_CRITICAL_ISR(&state_lock);

    if (!ui.armed) {
        BaseType_t higher_woken = pdFALSE;
        char cmd[] = "RELEASE_ALL";
        xQueueSendFromISR(line_queue, cmd, &higher_woken);
        if (higher_woken) portYIELD_FROM_ISR();
    }
}

// --- LCD 顏色渲染任務 ---
static void lcd_task(void *arg) {
    uint16_t *color_map = malloc(LCD_W * LCD_H * sizeof(uint16_t));
    if (!color_map) vTaskDelete(NULL);

    while (true) {
        bool armed;
        portENTER_CRITICAL(&state_lock);
        armed = ui.armed;
        portEXIT_CRITICAL(&state_lock);

        uint16_t fill_color = armed ? 0x07E0 : 0xF800;

        for (int i = 0; i < LCD_W * LCD_H; i++) {
            color_map[i] = fill_color;
        }

        esp_lcd_panel_draw_bitmap(panel, 0, 0, LCD_W, LCD_H, color_map);
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// --- 硬體初始化 ---
static void init_lcd(void) {
    gpio_config_t bl = { .pin_bit_mask = 1ULL << LCD_PIN_BL, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&bl);
    gpio_set_level(LCD_PIN_BL, 1);

    spi_bus_config_t bus = {
        .sclk_io_num = LCD_PIN_SCLK, .mosi_io_num = LCD_PIN_MOSI,
        .miso_io_num = -1, .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = LCD_W * LCD_H * 2,
    };
    spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO);

    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num = LCD_PIN_DC, .cs_gpio_num = LCD_PIN_CS,
        .pclk_hz = 40 * 1000 * 1000, .lcd_cmd_bits = 8, .lcd_param_bits = 8,
        .spi_mode = 0, .trans_queue_depth = 10,
    };
    esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io);

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = LCD_PIN_RST, .bits_per_pixel = 16,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    };
    esp_lcd_new_panel_st7789(io, &panel_cfg, &panel);
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_set_gap(panel, 40, 53); 
    esp_lcd_panel_invert_color(panel, true);
    esp_lcd_panel_disp_on_off(panel, true);
}

void app_main(void) {
    line_queue = xQueueCreate(16, CMD_LINE_MAX);
    init_lcd();

    tinyusb_config_t tusb_cfg = { 0 }; 
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t cdc_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = cdc_rx_cb,
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&cdc_cfg));

    gpio_config_t boot = {
        .pin_bit_mask = 1ULL << BOOT_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&boot);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_PIN, boot_isr, NULL);

    xTaskCreate(command_task, "command", 4096, NULL, 5, NULL);
    xTaskCreate(safety_task, "safety", 3072, NULL, 5, NULL);
    xTaskCreate(lcd_task, "lcd", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "自動化巨集終端啟動完成！");
}
