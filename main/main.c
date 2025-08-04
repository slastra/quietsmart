/**
 * @file main.c
 * @brief ESP32 PWM Fan Controller with WiFi/MQTT control
 * 
 * Features:
 * - PWM speed control (0-100%) on GPIO 6
 * - Button toggle control on GPIO 16
 * - MQTT remote control and status reporting
 * - mDNS for local network discovery
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/i2c.h"
#include "mqtt_client.h"
#include "mdns.h"
#include "esp_timer.h"
#include "esp_random.h"

// Configuration
#define CONFIG_WIFI_SSID            "SSID"
#define CONFIG_WIFI_PASSWORD        "PASSWORD"
#define CONFIG_MQTT_BROKER_URI      "mqtt://hostname.local:1883"
#define CONFIG_MQTT_STATUS_TOPIC    "fan/status"
#define CONFIG_MQTT_CONTROL_TOPIC   "fan/control"
#define CONFIG_FAN_PWM_PIN          GPIO_NUM_6
#define CONFIG_FAN_TACH_PIN         GPIO_NUM_7
#define CONFIG_FAN_BUTTON_PIN       GPIO_NUM_16
#define CONFIG_LED_PIN              GPIO_NUM_15
#define CONFIG_SHT31_SDA_PIN        GPIO_NUM_9
#define CONFIG_SHT31_SCL_PIN        GPIO_NUM_8
#define CONFIG_PWM_FREQUENCY        25000
#define CONFIG_STATUS_INTERVAL_MS   5000

// SHT31-D I2C Configuration
#define SHT31_I2C_ADDR              0x44
#define SHT31_CMD_MEASURE_HIGH      0x2C06
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// Constants
#define PWM_RESOLUTION         LEDC_TIMER_8_BIT
#define PWM_CHANNEL            LEDC_CHANNEL_0
#define PWM_TIMER              LEDC_TIMER_0
#define PWM_MODE               LEDC_LOW_SPEED_MODE
#define WIFI_CONNECTED_BIT     BIT0
#define MQTT_CONNECTED_BIT     BIT1

static const char *TAG = "FAN_CTRL";

// Forward declarations
static void set_fan_speed(uint8_t speed);
static esp_err_t save_fan_speed(uint8_t speed);
static esp_err_t init_i2c(void);
static esp_err_t sht31_read_data(float *temperature, float *humidity);

// Speed levels for button cycling
static const uint8_t speed_levels[] = {0, 25, 50, 75, 100};
static const int num_speed_levels = sizeof(speed_levels) / sizeof(speed_levels[0]);

// Global state
static struct {
    EventGroupHandle_t event_group;
    esp_mqtt_client_handle_t mqtt_client;
    esp_timer_handle_t status_timer;
    pcnt_unit_handle_t pcnt_unit;
    uint8_t fan_speed;      // Current speed 0-100%
    uint8_t speed_index;    // Current index in speed_levels array
    uint32_t fan_rpm;
    float temperature_c;    // Temperature in Celsius
    float humidity;         // Humidity percentage
} app_state = {0};

// --- NVS Storage Functions ---

static esp_err_t load_saved_speed(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("fan_config", NVS_READONLY, &nvs_handle);
    
    if (err == ESP_OK) {
        size_t required_size = sizeof(uint8_t);
        err = nvs_get_blob(nvs_handle, "last_speed", &app_state.fan_speed, &required_size);
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Loaded saved fan speed: %d%%", app_state.fan_speed);
            // Find the corresponding speed index
            for (int i = 0; i < num_speed_levels; i++) {
                if (speed_levels[i] == app_state.fan_speed) {
                    app_state.speed_index = i;
                    break;
                }
            }
            set_fan_speed(app_state.fan_speed);
        } else {
            // No saved speed, use default (50%)
            app_state.fan_speed = 50;
            app_state.speed_index = 2; // Index for 50% in speed_levels
            set_fan_speed(50);
            ESP_LOGI(TAG, "No saved speed found, using default: 50%%");
        }
        nvs_close(nvs_handle);
    } else {
        app_state.fan_speed = 50;
        app_state.speed_index = 2;
        set_fan_speed(50);
        ESP_LOGI(TAG, "Failed to open NVS, using default speed: 50%%");
    }
    
    return ESP_OK;
}

static esp_err_t save_fan_speed(uint8_t speed) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("fan_config", NVS_READWRITE, &nvs_handle);
    
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs_handle, "last_speed", &speed, sizeof(uint8_t));
        if (err == ESP_OK) {
            err = nvs_commit(nvs_handle);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Saved fan speed: %d%%", speed);
            }
        }
        nvs_close(nvs_handle);
    }
    
    return err;
}

// --- I2C and SHT31-D Functions ---

static esp_err_t init_i2c(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_SHT31_SDA_PIN,
        .scl_io_num = CONFIG_SHT31_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 
                            I2C_MASTER_RX_BUF_DISABLE, 
                            I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C initialized: SDA=%d, SCL=%d", CONFIG_SHT31_SDA_PIN, CONFIG_SHT31_SCL_PIN);
    return ESP_OK;
}

static uint8_t sht31_crc8(const uint8_t *data, int len) {
    const uint8_t polynomial = 0x31;
    uint8_t crc = 0xFF;
    
    for (int j = len; j; --j) {
        crc ^= *data++;
        for (int i = 8; i; --i) {
            crc = (crc & 0x80) ? (crc << 1) ^ polynomial : (crc << 1);
        }
    }
    return crc;
}

static esp_err_t sht31_read_data(float *temperature, float *humidity) {
    uint8_t cmd[2] = {(SHT31_CMD_MEASURE_HIGH >> 8) & 0xFF, SHT31_CMD_MEASURE_HIGH & 0xFF};
    uint8_t data[6];
    
    // Create I2C command link
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (SHT31_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd_handle, cmd, 2, true);
    i2c_master_stop(cmd_handle);
    
    // Send measurement command
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd_handle);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHT31 write failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Wait for measurement (high precision needs ~15ms)
    vTaskDelay(pdMS_TO_TICKS(20));
    
    // Read measurement data
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (SHT31_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd_handle, data, 5, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd_handle, &data[5], I2C_MASTER_NACK);
    i2c_master_stop(cmd_handle);
    
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd_handle);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SHT31 read failed: %s", esp_err_to_name(err));
        return err;
    }
    
    // Verify CRC for temperature
    if (sht31_crc8(data, 2) != data[2]) {
        ESP_LOGW(TAG, "SHT31 temperature CRC error");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Verify CRC for humidity
    if (sht31_crc8(data + 3, 2) != data[5]) {
        ESP_LOGW(TAG, "SHT31 humidity CRC error");
        return ESP_ERR_INVALID_CRC;
    }
    
    // Convert raw data to physical values
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t hum_raw = (data[3] << 8) | data[4];
    
    *temperature = -45.0f + 175.0f * temp_raw / 65535.0f;
    *humidity = 100.0f * hum_raw / 65535.0f;
    
    return ESP_OK;
}

// --- Hardware Control Functions ---

static esp_err_t init_pwm(void) {
    ledc_timer_config_t timer_cfg = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = CONFIG_PWM_FREQUENCY,
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    
    ledc_channel_config_t channel_cfg = {
        .channel = PWM_CHANNEL,
        .duty = 0,
        .gpio_num = CONFIG_FAN_PWM_PIN,
        .speed_mode = PWM_MODE,
        .hpoint = 0,
        .timer_sel = PWM_TIMER,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    
    ESP_LOGI(TAG, "PWM initialized: %dHz on GPIO%d", CONFIG_PWM_FREQUENCY, CONFIG_FAN_PWM_PIN);
    return ESP_OK;
}

static void set_fan_speed(uint8_t speed) {
    app_state.fan_speed = speed;
    uint32_t duty = (speed * 255) / 100;
    
    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    ESP_LOGI(TAG, "Speed: %d%%", speed);
}


static esp_err_t init_button(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_FAN_BUTTON_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Button on GPIO%d", CONFIG_FAN_BUTTON_PIN);
    return ESP_OK;
}

static inline bool is_button_pressed(void) {
    return gpio_get_level(CONFIG_FAN_BUTTON_PIN) == 0;
}

static esp_err_t init_tachometer(void) {
    // Configure GPIO for tachometer with pull-up for open-collector input
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CONFIG_FAN_TACH_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,  // Enable internal pull-up for open-collector
    };
    gpio_config(&io_conf);
    
    // Configure pulse counter
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &app_state.pcnt_unit));
    
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = CONFIG_FAN_TACH_PIN,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(app_state.pcnt_unit, &chan_config, &pcnt_chan));
    
    // Count on rising edge only
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
    
    ESP_ERROR_CHECK(pcnt_unit_enable(app_state.pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(app_state.pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(app_state.pcnt_unit));
    
    ESP_LOGI(TAG, "Tachometer on GPIO%d with pull-up enabled", CONFIG_FAN_TACH_PIN);
    return ESP_OK;
}

static uint32_t calculate_rpm(void) {
    static int last_count = 0;
    static uint64_t last_time = 0;
    
    int current_count;
    ESP_ERROR_CHECK(pcnt_unit_get_count(app_state.pcnt_unit, &current_count));
    
    uint64_t current_time = esp_timer_get_time();
    
    if (last_time == 0) {
        last_count = current_count;
        last_time = current_time;
        return 0;
    }
    
    uint64_t time_diff = current_time - last_time;
    
    // Only calculate if at least 1 second has passed
    if (time_diff >= 1000000) {
        int pulse_diff = current_count - last_count;
        
        // Handle counter overflow
        if (pulse_diff < 0) {
            pulse_diff += 65536;
        }
        
        // Calculate RPM: (pulses/second) * 60 / 2
        // Divide by 2 because PC fans output 2 pulses per revolution
        if (pulse_diff > 0 && time_diff > 0) {
            app_state.fan_rpm = (uint32_t)((uint64_t)pulse_diff * 30000000ULL / time_diff);
            
            // Sanity check - typical PC fans don't exceed 5000 RPM
            if (app_state.fan_rpm > 5000) {
                app_state.fan_rpm = 0;
            }
        } else {
            app_state.fan_rpm = 0;
        }
        
        last_count = current_count;
        last_time = current_time;
    }
    
    return app_state.fan_rpm;
}

static esp_err_t init_led(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << CONFIG_LED_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(CONFIG_LED_PIN, 0);
    return ESP_OK;
}

static void flash_led(void) {
    gpio_set_level(CONFIG_LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(CONFIG_LED_PIN, 0);
}

static void blink_led_count(int count) {
    for (int i = 0; i < count; i++) {
        gpio_set_level(CONFIG_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(200));  // LED on for 200ms
        gpio_set_level(CONFIG_LED_PIN, 0);
        
        // Don't add delay after the last blink
        if (i < count - 1) {
            vTaskDelay(pdMS_TO_TICKS(200));  // LED off for 200ms between blinks
        }
    }
}

// --- MQTT Functions ---

static void publish_status(void* arg) {
    // Update RPM calculation
    calculate_rpm();
    
    // Read temperature/humidity
    sht31_read_data(&app_state.temperature_c, &app_state.humidity);
    
    char msg[192];
    int len = snprintf(msg, sizeof(msg), 
        "{\"speed\":%d,\"rpm\":%lu,\"temp_c\":%.1f,\"humidity\":%.1f,\"timestamp\":%u}", 
        app_state.fan_speed,
        app_state.fan_rpm,
        app_state.temperature_c,
        app_state.humidity,
        (unsigned int)(esp_timer_get_time() / 1000000));
    
    esp_mqtt_client_publish(app_state.mqtt_client, CONFIG_MQTT_STATUS_TOPIC, msg, len, 1, 0);
}

static void handle_mqtt_command(const char* data, int len) {
    char cmd[256] = {0};
    int copy_len = (len < sizeof(cmd) - 1) ? len : sizeof(cmd) - 1;
    memcpy(cmd, data, copy_len);
    
    ESP_LOGI(TAG, "Command: %s", cmd);
    
    if (strstr(cmd, "\"speed\"")) {
        char* p = strstr(cmd, "\"speed\"");
        if (p) {
            // Find the colon and skip any spaces
            p = strchr(p, ':');
            if (p) {
                p++; // Skip the colon
                while (*p == ' ' || *p == '\t') p++; // Skip whitespace
                int speed = atoi(p);
                if (speed >= 0 && speed <= 100) {
                    set_fan_speed(speed);
                    save_fan_speed(speed);
                    flash_led();
                }
            }
        }
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            esp_mqtt_client_subscribe(app_state.mqtt_client, CONFIG_MQTT_CONTROL_TOPIC, 1);
            xEventGroupSetBits(app_state.event_group, MQTT_CONNECTED_BIT);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT Disconnected");
            xEventGroupClearBits(app_state.event_group, MQTT_CONNECTED_BIT);
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT Data received");
            handle_mqtt_command(event->data, event->data_len);
            break;
            
        default:
            break;
    }
}

// --- WiFi Functions ---

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(app_state.event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected, IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(app_state.event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_sta(void) {
    app_state.event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialization complete");
    return ESP_OK;
}

// --- Main Task ---

static void fan_control_task(void *pvParameters) {
    ESP_LOGI(TAG, "Waiting for network connection...");
    
    xEventGroupWaitBits(app_state.event_group, 
                       WIFI_CONNECTED_BIT | MQTT_CONNECTED_BIT, 
                       pdFALSE, pdTRUE, portMAX_DELAY);
    
    ESP_LOGI(TAG, "Network connected, initializing hardware...");
    
    init_pwm();
    init_button();
    init_tachometer();
    
    // Setup status timer
    const esp_timer_create_args_t timer_args = {
        .callback = &publish_status,
        .name = "status_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &app_state.status_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(app_state.status_timer, CONFIG_STATUS_INTERVAL_MS * 1000));
    
    // Initialize I2C for SHT31
    init_i2c();
    
    ESP_LOGI(TAG, "System ready");
    
    bool last_button_state = false;
    
    while (1) {
        bool current_button_state = is_button_pressed();
        
        // Button cycles through speed levels
        if (current_button_state && !last_button_state) {
            ESP_LOGI(TAG, "Button pressed - cycling speed");
            
            // Move to next speed level
            app_state.speed_index = (app_state.speed_index + 1) % num_speed_levels;
            uint8_t new_speed = speed_levels[app_state.speed_index];
            
            set_fan_speed(new_speed);
            save_fan_speed(new_speed);
            
            ESP_LOGI(TAG, "Speed cycled to: %d%% (level %d/%d)", new_speed, app_state.speed_index + 1, num_speed_levels);
            
            // Blink LED to indicate speed level (1-5 blinks)
            blink_led_count(app_state.speed_index + 1);
            
            vTaskDelay(pdMS_TO_TICKS(200));  // Debounce
        }
        
        last_button_state = current_button_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// --- Main Entry Point ---

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP32 Fan Controller Starting...");
    
    // Load saved fan speed
    load_saved_speed();
    
    // Initialize hardware
    init_led();
    
    // Initialize networking
    wifi_init_sta();
    
    // Initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp32-fan-controller"));
    
    // Initialize MQTT
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
    };
    app_state.mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(app_state.mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(app_state.mqtt_client);
    
    // Start main task
    xTaskCreate(fan_control_task, "fan_control_task", 4096, NULL, 10, NULL);
}
