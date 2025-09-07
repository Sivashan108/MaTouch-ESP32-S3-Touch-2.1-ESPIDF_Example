#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_lcd_panel_ops.h>
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_st7701.h"
#include <esp_log.h>
#include <freertos/task.h>
#include "driver/gpio.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"
#include <lvgl.h>
#include "lv_conf.h"
#include "esp_log.h"
#include <esp_timer.h>
#include "ui.h"
const char *TAG_LCD = "RGB_LCD";

// Pin definitions for RGB interface
#define PIN_NUM_HSYNC 3
#define PIN_NUM_VSYNC 42
#define PIN_NUM_DE 2
#define PIN_NUM_PCLK 45

// RGB Data Lines
#define PIN_NUM_R0 11
#define PIN_NUM_R1 15
#define PIN_NUM_R2 12
#define PIN_NUM_R3 16
#define PIN_NUM_R4 21

#define PIN_NUM_G0 39
#define PIN_NUM_G1 7
#define PIN_NUM_G2 47
#define PIN_NUM_G3 8
#define PIN_NUM_G4 48
#define PIN_NUM_G5 9

#define PIN_NUM_B0 4
#define PIN_NUM_B1 41
#define PIN_NUM_B2 5
#define PIN_NUM_B3 40
#define PIN_NUM_B4 6

#define PIN_NUM_RST -1
#define PIN_NUM_BCKL 38

#define LCD_WIDTH 480
#define LCD_HEIGHT 480
#define LCD_DATA_WIDTH 16
void set_background_color(lv_color_t color);
static const char *TAG = "TOUCH_DEBUG";
#define I2C_MASTER_SCL_IO 18      // GPIO for SCL
#define I2C_MASTER_SDA_IO 17      // GPIO for SDA
#define I2C_MASTER_NUM I2C_NUM_0  // I2C port number
#define I2C_MASTER_FREQ_HZ 100000 // I2C clock frequency
#define TOUCH_PANEL_ADDR 0x15     // CST826 I2C address
#define BUTTON_PIN 14
#define ENCODER_CLK 13
#define ENCODER_DT 10
TimerHandle_t lvgl_tick_timer;
static volatile int encoder_counter = 0;
static volatile bool rotation = false;
static volatile int last_state_clk = 0;
static volatile int current_state_clk = 0;
#define LV_TICK_PERIOD_MS 1 // Define the tick period (adjust as needed)
static volatile int last_logged_value = 0;

lv_obj_t *encoder_label; // Label object

static volatile bool encoder_needs_update = false; // Flag to indicate update
static char encoder_buf[32];                       // Buffer to store the updated label text
lv_obj_t *button_label;                            // Label for button press
static volatile bool button_needs_update = false;  // Flag to indicate label update
static char button_buf[32];                        // Buffer to store the button label text

/**
 * @brief Rotary Encoder main task
 *
 * @param arg 
 */
void rotary_encoder_task(void *arg) {
    while (1) {
        current_state_clk = gpio_get_level(ENCODER_CLK);

        // Detect rising edge on the CLK pin
        if (current_state_clk == 1 && last_state_clk == 0) {
            if (gpio_get_level(ENCODER_DT) != current_state_clk) {
                encoder_counter--; // Counter-clockwise rotation
            } else {
                encoder_counter++; // Clockwise rotation
            }
            rotation = true;

            // Log and update the encoder value when it changes
            if (encoder_counter != last_logged_value) {
                ESP_LOGI(TAG, "Encoder value changed: %d", encoder_counter);
                last_logged_value = encoder_counter; // Update the last logged value

                // Store the new encoder position in the buffer
                snprintf(encoder_buf, sizeof(encoder_buf), "Encoder Position: %d", encoder_counter);
                encoder_needs_update = true; // Set the flag to update the label in the LVGL task
            }
        }
        last_state_clk = current_state_clk;

        // Small delay for debouncing
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/**
 * @brief Button monitoring task
 * 
 * @param arg 
 */
void button_task(void *arg) {
    bool last_button_state = gpio_get_level(BUTTON_PIN);

    while (1) {
        bool current_button_state = gpio_get_level(BUTTON_PIN);

        // Check for button press (active low, assuming 0 means pressed)
        if (current_button_state != last_button_state) {
            if (current_button_state == 0) {
                snprintf(button_buf, sizeof(button_buf), "Button Pressed");
            } else {
                snprintf(button_buf, sizeof(button_buf), "Button Released");
            }

            button_needs_update = true; // Set flag to update the label in LVGL task
            last_button_state = current_button_state; // Update last state
        }

        // Small delay for debouncing
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
static void lv_tick_task(void *arg)
{
    lv_tick_inc(LV_TICK_PERIOD_MS); // Increase the LVGL tick
}

// Set the timer to call lv_tick_task every LV_TICK_PERIOD_MS
esp_timer_create_args_t tick_timer_args = {
    .callback = &lv_tick_task,
    .name = "lv_tick"};

/** 
 * @brief Encoder read callback for LVGL
 * @param drv Pointer to the input device driver
 * @param data Pointer to the input device data structure
 * @returns void
 */
static void encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static int last_enc_value = 0;

    // Check if rotation occurred
    if (rotation)
    {
        if (encoder_counter > last_enc_value)
        {
            data->key = LV_KEY_RIGHT; // Clockwise rotation
        }
        else
        {
            data->key = LV_KEY_LEFT; // Counter-clockwise rotation
        }
        data->state = LV_INDEV_STATE_PR;
        last_enc_value = encoder_counter;
        rotation = false;
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }

    // Handle button presses
    if (gpio_get_level(BUTTON_PIN) == 0)
    {
        data->state = LV_INDEV_STATE_PR;
        data->key = LV_KEY_ENTER; // Simulate enter key press
    }
}

/**
 * @brief i2c master initialization
 * @param void
 * @return void
 * 
 */
void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO, // Define SDA pin
        .scl_io_num = I2C_MASTER_SCL_IO, // Define SCL pin
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // Set clock speed (e.g., 100kHz)
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}
static const char *TAG_lvgl = "LVGL";

/**
 * @brief Logging function for LVGL
 * 
 * @param buf 
 */
void lv_esp_log(const char *buf)
{
    ESP_LOGI(TAG_lvgl, "%s", buf);
}

/**
 * @brief I2C read function with retries
 * 
 * @param addr I2C device address
 * @param reg_addr Register address to read from
 * @param reg_data Pointer to buffer to store read data
 * @param length Number of bytes to read
 * @return esp_err_t ESP_OK on success, error code on failure
 */
esp_err_t i2c_read(uint16_t addr, uint8_t reg_addr, uint8_t *reg_data, uint32_t length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    for (int retry = 0; retry < 3; retry++)
    { // Retry up to 3 times
        // ESP_LOGI(TAG, "Starting I2C communication to 0x%02X, attempt %d", addr, retry+1);

        // Start I2C communication
        ret = i2c_master_start(cmd);
        if (ret != ESP_OK)
        {
            // ESP_LOGE(TAG, "I2C start failed: %s", esp_err_to_name(ret));
            continue;
        }

        // Send the device address and register to read from
        ret = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        if (ret != ESP_OK)
        {
            // ESP_LOGE(TAG, "I2C write byte (address) failed: %s", esp_err_to_name(ret));
            continue;
        }

        ret = i2c_master_write_byte(cmd, reg_addr, true);
        if (ret != ESP_OK)
        {
            // ESP_LOGE(TAG, "I2C write byte (register) failed: %s", esp_err_to_name(ret));
            continue;
        }

        // Send repeated start for reading
        ret = i2c_master_start(cmd);
        if (ret != ESP_OK)
        {
            // ESP_LOGE(TAG, "I2C repeated start failed: %s", esp_err_to_name(ret));
            continue;
        }

        ret = i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
        if (ret != ESP_OK)
        {
            // ESP_LOGE(TAG, "I2C write byte (read mode) failed: %s", esp_err_to_name(ret));
            continue;
        }

        // Read the data
        if (length > 1)
        {
            ret = i2c_master_read(cmd, reg_data, length - 1, I2C_MASTER_ACK);
            if (ret != ESP_OK)
            {
                // ESP_LOGE(TAG, "I2C master read failed: %s", esp_err_to_name(ret));
                continue;
            }
        }
        ret = i2c_master_read_byte(cmd, reg_data + length - 1, I2C_MASTER_NACK);
        if (ret != ESP_OK)
        {
            // ESP_LOGE(TAG, "I2C master read byte failed: %s", esp_err_to_name(ret));
            continue;
        }

        // Stop I2C communication
        ret = i2c_master_stop(cmd);
        if (ret != ESP_OK)
        {
            // ESP_LOGE(TAG, "I2C stop failed: %s", esp_err_to_name(ret));
            continue;
        }

        // Execute I2C command
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(2000)); // Increase timeout to 2 seconds
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            // ESP_LOGI(TAG, "I2C read successful");
            break; // Exit the retry loop on success
        }
        else
        {
            // ESP_LOGE(TAG, "I2C command failed: %s", esp_err_to_name(ret));
        }
    }

    return ret;
}

/**
 * @brief Read touch data from the touch panel
 * 
 * @param x Pointer to store the X coordinate
 * @param y Pointer to store the Y coordinate
 * @return int 1 if touch detected, 0 otherwise
 */
int read_touch(int *x, int *y)
{
    uint8_t data_raw[7];                                           // Buffer to store touch data
    esp_err_t ret = i2c_read(TOUCH_PANEL_ADDR, 0x02, data_raw, 7); // 0x02 is the register address to read touch data

    if (ret != ESP_OK)
    {
        // ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return 0;
    }

    // Print raw data for debugging
    // ESP_LOGI(TAG, "Touch raw data: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X",
    //  data_raw[0], data_raw[1], data_raw[2], data_raw[3], data_raw[4], data_raw[5], data_raw[6]);

    int event = data_raw[1] >> 6; // Get touch event type (2 = touch)
    if (event == 2)
    {                                                            // Touch detected
        *x = (int)data_raw[2] + (int)(data_raw[1] & 0x0F) * 256; // X coordinate
        *y = (int)data_raw[4] + (int)(data_raw[3] & 0x0F) * 256; // Y coordinate

        // ESP_LOGI(TAG, "Touch detected at X: %d, Y: %d", *x, *y);
        return 1;
    }
    else
    {
        // ESP_LOGI(TAG, "No touch detected");
        return 0;
    }
}

/**
 * @brief Touchpad read callback for LVGL
 * 
 * @param indev_driver Pointer to the input device driver
 * @param data Pointer to the input device data structure
 */
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    static int last_touchX = -1, last_touchY = -1;
    int touchX = 0, touchY = 0;

    if (read_touch(&touchX, &touchY) == 1)
    {
        if (abs(touchX - last_touchX) > 2 || abs(touchY - last_touchY) > 2)
        {                                    // Only update if there is significant movement
            data->state = LV_INDEV_STATE_PR; // Touch pressed
            data->point.x = (uint16_t)touchX;
            data->point.y = (uint16_t)touchY;
            last_touchX = touchX;
            last_touchY = touchY;
            ESP_LOGI(TAG, "LVGL touch event: X: %d, Y: %d", touchX, touchY);
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_REL; // Touch released
    }
}
/**
 * @brief Scan the I2C bus for connected devices
 * 
 */
void i2c_scanner()
{
    printf("Scanning I2C bus...\n");
    for (int addr = 1; addr < 127; addr++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK)
        {
            printf("Device found at address: 0x%02X\n", addr);
        }
    }
    printf("I2C scan completed.\n");
}

spi_line_config_t line_config = {
    .cs_io_type = IO_TYPE_GPIO,
    .cs_gpio_num = 1,
    .scl_io_type = IO_TYPE_GPIO,
    .scl_gpio_num = 46,
    .sda_io_type = IO_TYPE_GPIO,
    .sda_gpio_num = 0,
    .io_expander = NULL,
};
lv_disp_drv_t disp_drv;
static const st7701_lcd_init_cmd_t init_cmds[] = {
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10}, 5},
    {0xC0, (uint8_t[]){0x3B, 0x00}, 2},
    {0xC1, (uint8_t[]){0x0B, 0x02}, 2},
    {0xC2, (uint8_t[]){0x00, 0x02}, 2},
    {0xCC, (uint8_t[]){0x10}, 1},
    {0xCD, (uint8_t[]){0x08}, 1},

    {0xB0, (uint8_t[]){0x02, 0x13, 0x1B, 0x0D, 0x10, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0E, 0x2C, 0x33, 0x1D}, 16},
    {0xB1, (uint8_t[]){0x05, 0x13, 0x1B, 0x0D, 0x11, 0x05, 0x08, 0x07, 0x07, 0x24, 0x04, 0x11, 0x0E, 0x2C, 0x33, 0x1D}, 16},

    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5},
    {0xB0, (uint8_t[]){0x5D}, 1},
    {0xB1, (uint8_t[]){0x43}, 1},
    {0xB2, (uint8_t[]){0x81}, 1},
    {0xB3, (uint8_t[]){0x80}, 1},
    {0xB5, (uint8_t[]){0x43}, 1},
    {0xB7, (uint8_t[]){0x85}, 1},
    {0xB8, (uint8_t[]){0x20}, 1},

    {0xC1, (uint8_t[]){0x78}, 1},
    {0xC2, (uint8_t[]){0x78}, 1},
    {0xD0, (uint8_t[]){0x88}, 1},

    {0xE0, (uint8_t[]){0x00, 0x00, 0x02}, 3},
    {0xE1, (uint8_t[]){0x03, 0xA0, 0x00, 0x00, 0x04, 0xA0, 0x00, 0x00, 0x00, 0x20, 0x20}, 11},
    {0xE2, (uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 13},
    {0xE3, (uint8_t[]){0x00, 0x00, 0x11, 0x00}, 4},
    {0xE4, (uint8_t[]){0x22, 0x00}, 2},
    {0xE5, (uint8_t[]){0x05, 0xEC, 0xA0, 0xA0, 0x07, 0xEE, 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16},
    {0xE6, (uint8_t[]){0x00, 0x00, 0x11, 0x00}, 4},
    {0xE7, (uint8_t[]){0x22, 0x00}, 2},
    {0xE8, (uint8_t[]){0x06, 0xED, 0xA0, 0xA0, 0x08, 0xEF, 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 16},
    {0xEB, (uint8_t[]){0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 7},
    {0xED, (uint8_t[]){0xFF, 0xFF, 0xFF, 0xBA, 0x0A, 0xBF, 0x45, 0xFF, 0xFF, 0x54, 0xFB, 0xA0, 0xAB, 0xFF, 0xFF, 0xFF}, 16},
    {0xEF, (uint8_t[]){0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F}, 6},

    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5},
    {0xEF, (uint8_t[]){0x08}, 1},

    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5},
    // {0x36, (uint8_t[]){0x00}, 1},
    {0x3A, (uint8_t[]){0x60}, 1}, // RGB666
    {0x11, NULL, 0, 120},         // Exit sleep mode
    {0x29, NULL, 0, 20},          // Display ON
};

/**
 * @brief event callback for VSYNC
 * 
 * @param panel 
 * @param event_data 
 * @param user_data 
 * @return true 
 * @return false 
 */
static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    return high_task_awoken == pdTRUE;
}

/**
 * @brief Function to draw "Hello, World!" on the LCD
 * 
 * @param panel_handle Handle to the LCD panel
 */
void draw_hello_world(esp_lcd_panel_handle_t panel_handle);
// RGB Panel Configuration
esp_lcd_rgb_panel_config_t rgb_config = {
    .data_width = 16,
    .bits_per_pixel = 16,
    .psram_trans_align = 64,
    .num_fbs = 2,
    .bounce_buffer_size_px = 10 * LCD_WIDTH,
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .disp_gpio_num = GPIO_NUM_NC,
    .pclk_gpio_num = PIN_NUM_PCLK,
    .vsync_gpio_num = PIN_NUM_VSYNC,
    .hsync_gpio_num = PIN_NUM_HSYNC,
    .de_gpio_num = PIN_NUM_DE,
    .disp_gpio_num = -1,
    .data_gpio_nums = {
        PIN_NUM_R0, PIN_NUM_R1, PIN_NUM_R2, PIN_NUM_R3, PIN_NUM_R4,
        PIN_NUM_G0, PIN_NUM_G1, PIN_NUM_G2, PIN_NUM_G3, PIN_NUM_G4, PIN_NUM_G5,

        PIN_NUM_B0, PIN_NUM_B1, PIN_NUM_B2, PIN_NUM_B3, PIN_NUM_B4

    },
    .timings = {
        .pclk_hz = 8 * 1000 * 1000,
        .h_res = LCD_WIDTH,
        .v_res = LCD_HEIGHT,
        .hsync_back_porch = 50,
        .hsync_front_porch = 6,
        .hsync_pulse_width = 8,
        .vsync_back_porch = 20,
        .vsync_front_porch = 10,
        .vsync_pulse_width = 8,

    },

    .flags = {
        .fb_in_psram = true,
    },
};

// Vendor configuration
st7701_vendor_config_t vendor_config = {
    .rgb_config = &rgb_config,
    .init_cmds = init_cmds, // Make sure these are valid commands
    .init_cmds_size = sizeof(init_cmds) / sizeof(st7701_lcd_init_cmd_t),
    .flags = {
        .use_mipi_interface = false,
        .enable_io_multiplex = false,
        .auto_del_panel_io = true, // Ensure proper cleanup after I/O deletion
    },
};

// Panel device configuration
const esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = PIN_NUM_RST,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
    .bits_per_pixel = 16,
    .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
    .vendor_config = &vendor_config,
};
esp_lcd_panel_handle_t panel_handle = NULL;
/**
 * @brief initialize the RGB LCD
 * 
 */
void init_rgb_lcd(void)
{

    // Initialize the RGB panel
    esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BCKL, 1); // Turn on backlight

    ESP_LOGI(TAG_LCD, "RGB Config Data Width: %d", rgb_config.data_width);
    ESP_LOGI(TAG_LCD, "Install ST7701 RGB panel");

    esp_err_t ret = esp_lcd_new_panel_st7701(io_handle, &panel_config, &panel_handle); // Pass NULL for `io` in RGB mode
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_LCD, "Failed to create new panel ST7701: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG_LCD, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // // Define a smaller chunk size to avoid large buffer allocation
    // const int chunk_height = 60;  // Height of each chunk (adjust as needed)
    // const int chunk_width = LCD_WIDTH;
    // uint32_t color = 0xfda0;
    // size_t chunk_size = chunk_width * chunk_height;

    // // Allocate a smaller buffer for a chunk of the screen
    // uint16_t *buffer = (uint16_t *)heap_caps_malloc(chunk_size * sizeof(uint16_t), MALLOC_CAP_DMA);
    // if (!buffer) {
    //     ESP_LOGE(TAG_LCD, "Failed to allocate memory for buffer");
    //     return;
    // }

    // // Fill the buffer with red color
    // for (size_t i = 0; i < chunk_size; i++) {
    //     buffer[i] = color;
    // }

    // // Draw the screen in chunks
    // for (int y = 0; y < LCD_HEIGHT; y += chunk_height) {
    //     ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, y, chunk_width, y + chunk_height, buffer));
    // }

    // // Free the buffer after drawing
    // heap_caps_free(buffer);
}

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;

// Display flush function (sending the buffer to the display)
/**
 * @brief Flush the display with the content of the buffer
 * 
 * @param disp_drv Pointer to the display driver
 * @param area Pointer to the area to be flushed
 * @param color_p Pointer to the color buffer
 * @return void
 */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    // Check the area is valid
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)disp_drv->user_data;

    // Pass the buffer data to the panel
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    ESP_LOGI(TAG, "Redrawing area: x1=%d, y1=%d, x2=%d, y2=%d", area->x1, area->y1, area->x2, area->y2);
    // Mark flush as complete
    lv_disp_flush_ready(disp_drv);
}
/**
 * @brief Round the area to be flushed to align with 8-pixel boundaries
 * 
 * @param disp_drv Pointer to the display driver
 * @param area Pointer to the area to be rounded
 */
void my_rounder_cb(lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    // Align the y1 and y2 coordinates to 8-pixel boundaries (rounding down)
    area->y1 = area->y1 & ~0x07;       // y1 aligned to nearest lower multiple of 8
    area->y2 = (area->y2 & ~0x07) + 7; // y2 aligned to nearest upper multiple of 8
}

int counter = 0;
int move_flag = 0;
/**
 * @brief Encoder read callback for LVGL
 * 
 * @param drv Pointer to the input device driver
 * @param data Pointer to the input device data structure
 */
void my_encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    if (counter > 0)
    {
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_LEFT;
        counter--;
        move_flag = 1;
    }

    else if (counter < 0)
    {
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_RIGHT;
        counter++;
        move_flag = 1;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }

    // if (enc_pressed())
    //     data->state = LV_INDEV_STATE_PRESSED;
    // else
    //     data->state = LV_INDEV_STATE_RELEASED;
}

/**
 * @brief Initialize LVGL display and input devices
 * 
 */
void init_lvgl_display()
{
    printf("Here1\n");
    void *buf2 = NULL;
    buf = heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    buf2 = heap_caps_malloc(LCD_WIDTH * LCD_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);

    // buf = heap_caps_malloc(LCD_WIDTH * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!buf)
    {
        ESP_LOGE(TAG_LCD, "Failed to allocate LVGL buffer");
        return;
    }
    lv_disp_draw_buf_init(&draw_buf, buf, buf2, LCD_WIDTH * LCD_HEIGHT);
    printf("Here3\n");

    // Initialize LVGL display driver
    // Initialize LVGL display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;      // Set horizontal resolution
    disp_drv.ver_res = LCD_HEIGHT;     // Set vertical resolution
    disp_drv.flush_cb = my_disp_flush; // Set the flush callback
    // disp_drv.rounder_cb = my_rounder_cb;  // Set the rounder callback
    disp_drv.draw_buf = &draw_buf; // Assign the buffer

    // Pass the panel handle to user_data field
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv); // Register the display driver

    ESP_LOGI(TAG, "LVGL display driver registered");

    // Initialize touchpad input
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);          // Initialize the driver
    indev_drv.type = LV_INDEV_TYPE_POINTER; // Set driver type to pointer
    indev_drv.disp = disp;
    indev_drv.read_cb = my_touchpad_read; // Set the touchpad read callback

    lv_indev_t *indev = lv_indev_drv_register(&indev_drv); // Register input device
    if (indev)
    {
        ESP_LOGI(TAG, "Touchpad driver registered successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Touchpad driver registration failed");
    }

    // encoder
    static lv_indev_drv_t indev_drv2;
    lv_indev_drv_init(&indev_drv2);
    indev_drv2.type = LV_INDEV_TYPE_ENCODER;
    indev_drv2.read_cb = my_encoder_read;
    lv_indev_t *encoder_indev = lv_indev_drv_register(&indev_drv2);

    if (encoder_indev)
    {
        ESP_LOGI(TAG, "Touchpad driver registered successfully");
    }
    else
    {
        ESP_LOGE(TAG, "Touchpad driver registration failed");
    }
}
/**
 * @brief Create a "Hello World" label in the center of the screen
 * 
 */
void create_hello_world_label()
{
    // Create a label and set its text
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello World");

    // Create a new style for the label
    static lv_style_t style;
    lv_style_init(&style);

    // Set the text size in the style
    lv_style_set_text_font(&style, &lv_font_montserrat_46); // Use 24px Montserrat font

    // Apply the style to the label
    lv_obj_add_style(label, &style, 0);

    // Center the label on the screen
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}
/**
 * @brief LVGL task to handle LVGL tasks and update labels
 * 
 * @param pvParameter Pointer to task parameters (not used)
 */
void lvgl_task(void *pvParameter) {
    while (1) {
        lv_timer_handler();  // Handle LVGL tasks

        // Update encoder label
        if (encoder_needs_update && encoder_label != NULL) {
            lv_label_set_text(encoder_label, encoder_buf);
            encoder_needs_update = false;
        }

        // Update button label
        if (button_needs_update && button_label != NULL) {
            lv_label_set_text(button_label, button_buf);
            button_needs_update = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // Adjust delay as needed
    }
}

static bool button_pressed = false; // Global variable to track button press stat
/**
 * @brief Event callback for the touch test button
 * 
 * @param e Pointer to the LVGL event
 */
void touch_button_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *button = lv_event_get_target(e);

    if (code == LV_EVENT_PRESSED)
    {
        // Toggle button color on each press
        button_pressed = !button_pressed;

        if (button_pressed)
        {
            lv_obj_set_style_bg_color(button, lv_color_hex(0xFF0000), LV_PART_MAIN); // Set button to red
            ESP_LOGI(TAG, "Button Pressed: Red");
        }
        else
        {
            lv_obj_set_style_bg_color(button, lv_color_hex(0x0000FF), LV_PART_MAIN); // Set button to blue
            ESP_LOGI(TAG, "Button Pressed: Blue");
        }
    }
}
/**
 * @brief Create a touch test button on the screen
 * 
 */
void create_touch_test_button()
{
    lv_obj_t *btn = lv_btn_create(lv_scr_act());
    lv_obj_set_pos(btn, 180, 350); // Manually set the button position

    lv_obj_set_size(btn, 100, 50); // Set button size

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, "Touch Me");

    // Make sure you don't call lv_obj_align on the label repeatedly
    lv_obj_center(label); // Align the label in the center of the button

    // Log when the button is pressed
    lv_obj_add_event_cb(btn, touch_button_event_cb, LV_EVENT_ALL, NULL);
}

/**
 * @brief Create a label to display the encoder position
 * 
 */
void create_encoder_label(void)
{
    // Create a new label and set its initial text
    encoder_label = lv_label_create(lv_scr_act());
    lv_label_set_text(encoder_label, "Encoder Position: 0");
    static lv_style_t style;
    lv_style_init(&style);

    // Set the text size in the style
    lv_style_set_text_font(&style, &lv_font_montserrat_40); // Use 24px Montserrat font

    // Apply the style to the label
    lv_obj_add_style(encoder_label, &style, 0);
    // Position the label on the screen
    lv_obj_align(encoder_label, LV_ALIGN_CENTER, 0, 50); // Align the label to the center
}
/**
 * @brief Create a label to display the button state
 * 
 */
void create_button_label(void)
{
    // Create a new label and set its initial text
    button_label = lv_label_create(lv_scr_act());
    lv_label_set_text(button_label, "Button Released");
    static lv_style_t style;
    lv_style_init(&style);

    // Set the text size in the style
    lv_style_set_text_font(&style, &lv_font_montserrat_40); // Use 24px Montserrat font

    // Apply the style to the label
    lv_obj_add_style( button_label , &style, 0);
    // Position the label on the screen
    // Position the label below the encoder label on the screen
    lv_obj_align(button_label, LV_ALIGN_CENTER, 0,-50); // Offset the label below the encoder label
}
/**
 * @brief Main application entry point
 * 
 */
void app_main(void)
{
    lv_log_register_print_cb(lv_esp_log);
    i2c_master_init();
    i2c_scanner();
    ESP_LOGI(TAG_LCD, "Initialize RGB LCD");
    init_rgb_lcd();
    ESP_LOGI(TAG_LCD, "RGB LCD initialized successfully");
    // draw_hello_world(panel_handle);
    lv_init();

    // Initialize the display and LVGL buffer
    init_lvgl_display();
    create_encoder_label();
    create_button_label();
    xTaskCreate(rotary_encoder_task, "rotary_encoder_task", 4096, NULL, 10, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 12, NULL);
    set_background_color(lv_color_hex(0xffa600)); // Blue color
    // Create "Hello World" label in the center of the screen
    create_hello_world_label();
    printf("Here8\n");
    create_touch_test_button();
    // ui_init(); // commented this out as this was used for a slider example using eez studio for future project
    // Create LVGL task to periodically refresh the display

    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LV_TICK_PERIOD_MS * 1000)); // Periodic task
    xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 2 * 4096, NULL, 5, NULL, 1);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("Here99\n");
}
/**
 * @brief Draw "Hello, World!" in the center of the LCD
 * 
 * @param panel_handle Handle to the LCD panel
 */
void draw_hello_world(esp_lcd_panel_handle_t panel_handle)
{
    // For simplicity, we're using a fake width/height for the 'Hello World' string.
    int text_width = 200; // Approximate pixel width for "Hello World"
    int text_height = 80; // Approximate pixel height for "Hello World"

    // Calculate the top-left corner of the text so that it is centered
    int x_pos = (LCD_WIDTH - text_width) / 2;
    int y_pos = (LCD_HEIGHT - text_height) / 2;

    // Here, you could render a bitmap or fill a buffer with  text pixel data
    // Instead, let's use a simple color box as a placeholder for the text.
    uint32_t text_color = 0x0000; // Black color (you can use other colors)
    size_t buffer_size = text_width * text_height;
    uint16_t *buffer = (uint16_t *)heap_caps_malloc(buffer_size * sizeof(uint16_t), MALLOC_CAP_DMA);

    if (!buffer)
    {
        ESP_LOGE(TAG_LCD, "Failed to allocate memory for text buffer");
        return;
    }

    // Fill the buffer with the text color (a simple white rectangle for now)
    for (size_t i = 0; i < buffer_size; i++)
    {
        buffer[i] = text_color;
    }

    // Draw the text (currently a rectangle) in the center
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, x_pos, y_pos, x_pos + text_width, y_pos + text_height, buffer));

    // Free the buffer after drawing
    heap_caps_free(buffer);
}
/**
 * @brief Set the background color of the active screen
 * 
 * @param color The color to set as background
 */
void set_background_color(lv_color_t color)
{
    // Get the active screen object
    lv_obj_t *scr = lv_scr_act();

    // Set the background color for the active screen
    lv_obj_set_style_bg_color(scr, color, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
}
