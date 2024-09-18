
// This code prints the value with the timing stored on the RTC initially and it waits for the RTC to be synchronized with the NTP before pushing the data to the Topic.
// If the battery is removed from the RTC and then It is powerd ON the it will not push the garbage initial value and wait for the synchronization to happen  before pushing the data to the Topic.
// The Distance is calculated using HC-SR04  ultrasonic sensor and the data is pushed to the topic with the timestamp of the data.
// MPU6050 Failure is added to check if the sensor is working properly or not.





#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "time.h"
#include "esp_sntp.h"
#include "mpu6050.h"
#include "ultrasonic.h"
#include "ds3231.h"
#include "User_Variables.h"
#include "protocol_examples_common.h"


#define WIFI_SSID "TP-Link_4548"
#define WIFI_PASSWORD "16358874"

#define I2C_PORT I2C_NUM_0
#define I2C_PORT_RTC I2C_NUM_1
#define RTC_SLAVE_ADDR 0x68     // RTC I2C address
#define MPU6050_SLAVE_ADDR 0x69 // MPU6050 I2C address
#define SDA_PIN 16
#define SCL_PIN 4
#define SDA_PIN_RTC 23
#define SCL_PIN_RTC 22

#define SAMPLE_INTERVAL_MS 100
#define DISPLAY_INTERVAL_MS 1000

#define MAX_DISTANCE_CM 500 // 5m max

#define TRIGGER_GPIO 18
#define ECHO_GPIO 19


uint8_t count = 0;
static float total_roll = 0;
static const char *TAG = "MQTT_MPU6050_RTC";

static float global_roll = 0;
static float global_pitch = 0;
static float distance = 0;
static mpu6050_handle_t sensor = NULL;

complimentary_angle_t angle;
mpu6050_acce_value_t acce_value;
mpu6050_gyro_value_t gyro_value;

bool wifi_conn_flag = false;
bool time_sync = false;
bool MPU6050_FAIL = false;
bool RTC_FAIL = false;

// Convert decimal to BCD   
uint8_t DecimalToBCD(int val)
{
    return ((val / 10) << 4) | (val % 10);
}

/* MQTT related functions and variables */
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
// MQTT Event handler
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, "/LIPL/sensordata/sensors", "Connected", 0, 1, 0);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, "/LIPL/sensordata/sensors", "Subscribed", 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
// MQTT App start
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

// Set time in DS3231
void setRTCFromUnixTime(time_t unix_time)
{
    struct tm *timeinfo = gmtime(&unix_time);
    uint8_t data_write[8];
    data_write[0] = 0x00;
    data_write[1] = DecimalToBCD(timeinfo->tm_sec);
    data_write[2] = DecimalToBCD(timeinfo->tm_min);
    data_write[3] = DecimalToBCD(timeinfo->tm_hour);
    data_write[5] = DecimalToBCD(timeinfo->tm_mday);
    data_write[6] = DecimalToBCD(timeinfo->tm_mon + 1);
    data_write[7] = DecimalToBCD(timeinfo->tm_year - 100);

    esp_err_t err = i2c_master_write_to_device(I2C_PORT_RTC, RTC_SLAVE_ADDR, data_write, sizeof(data_write), pdMS_TO_TICKS(1000));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set time to DS3231: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "RTC time set successfully");
    }
}

// Function to initialize SNTP and set the time from the NTP server
void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

// Function to wait until the time is set and update the RTC
void obtain_time_and_update_rtc(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};

    long int retry = 0;
    long const int retry_count = 345600;
    while (timeinfo.tm_year < (2024 - 1900) && ++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%ld/%ld)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry == retry_count)
    {
        ESP_LOGE(TAG, "Failed to synchronize time with NTP");
    }
    else
    {
        ESP_LOGI(TAG, "Time synchronized successfully");
        setRTCFromUnixTime(now);
        time_sync = true;
    }
}

// Wi-Fi event handler
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Reconnecting to Wi-Fi...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: %s", ip4addr_ntoa(&event->ip_info.ip));
        wifi_conn_flag = true;

        initialize_sntp();
        obtain_time_and_update_rtc();
    }
}

// Initialize Wi-Fi
void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}
// Initialize the ESP32 I2C bus and MPU6050
void i2c_EspMpu_init(i2c_port_t i2c, uint32_t freq)
{
    i2c_config_t conf;
    memset(&conf, 0, sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;

    i2c_param_config(i2c, &conf);
    i2c_driver_install(i2c, I2C_MODE_MASTER, 0, 0, 0);

    sensor = mpu6050_create(i2c, MPU6050_SLAVE_ADDR);
    mpu6050_config(sensor, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(sensor);

    return;
}

// Initialize DS3231
void init_ds3231(i2c_port_t i2cc, uint32_t freqq)
{
    i2c_config_t rtc;
    memset(&rtc, 0, sizeof(i2c_config_t));
    rtc.mode = I2C_MODE_MASTER;
    rtc.sda_io_num = SDA_PIN_RTC;
    rtc.scl_io_num = SCL_PIN_RTC;
    rtc.sda_pullup_en = GPIO_PULLUP_ENABLE;
    rtc.scl_pullup_en = GPIO_PULLUP_ENABLE;
    rtc.master.clk_speed = freqq;

    i2c_param_config(i2cc, &rtc);
    i2c_driver_install(i2cc, I2C_MODE_MASTER, 0, 0, 0);

    return;
}

// Timer callback function
void timer_callback(void)
{
    // Read accelerometer and gyroscope data
    mpu6050_get_acce(sensor, &acce_value);
    mpu6050_get_gyro(sensor, &gyro_value);

    // Calculate roll and pitch using complementary filter
    mpu6050_complimentory_filter(sensor, &acce_value, &gyro_value, &angle);

    //    Update global roll and pitch values
    global_roll = angle.roll;
    global_pitch = angle.pitch;
}

// Task for reading the MPU6050 sensor data
void mpu6050_task(void *pvParameter)
{
    while (1)
    {
        timer_callback();
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

// Convert BCD to decimal
int BCDToDecimal(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}
// Read Unix Time
time_t readUnixTimeFromDS3231()
{
    uint8_t size = 7;
    uint8_t data_read[7] = {0};
    uint8_t start_address = 0x00; // Starting register address

    esp_err_t err = i2c_master_write_read_device(I2C_PORT_RTC, RTC_SLAVE_ADDR, &start_address, 1, data_read, size, pdMS_TO_TICKS(1000));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read time from DS3231: %s", esp_err_to_name(err));
        return -1;
    }

    struct tm timeinfo;
    timeinfo.tm_sec = BCDToDecimal(data_read[0]);
    timeinfo.tm_min = BCDToDecimal(data_read[1]);
    timeinfo.tm_hour = BCDToDecimal(data_read[2]);
    timeinfo.tm_mday = BCDToDecimal(data_read[4]);
    timeinfo.tm_mon = BCDToDecimal(data_read[5]) - 1;    // tm_mon is 0-based
    timeinfo.tm_year = BCDToDecimal(data_read[6]) + 100; // tm_year is years since 1900
    timeinfo.tm_isdst = -1;                              // Disable daylight saving time

    return mktime(&timeinfo);
}

// Task for publishing sensor data via MQTT
void mqtt_publish_task(void *pvParameter)
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)pvParameter;
    char msg[300];

    while (1)
    {
        if (time_sync == true)
        {
            // Read unix time from DS3231
            time_t ds3231_time = readUnixTimeFromDS3231();
            // Variables to store accumulated pitch and roll values
            float total_roll = 0, total_pitch = 0;
            int sample_count = 0;

            for (int i = 0; i < 10; i++)
            {
                // Accumulate global roll and pitch values
                total_roll += global_roll;
                total_pitch += global_pitch;
                sample_count++;

                // Delay between each sample
                vTaskDelay(SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
            }

            // Calculate the average roll and pitch
            float avg_roll = total_roll / sample_count;
            float avg_pitch = total_pitch / sample_count;

            // snprintf(msg, sizeof(msg), "Roll: %.2f, Pitch: %.2f", avg_pitch, avg_roll);
            snprintf(msg, sizeof(msg), "{\"ID\": \"66b5e59a388d38768bb00a0d\",\"Time\": %lld,\"Roll\": %.2f, \"Pitch\": %.2f, \"Distance\": %0.02f}", (long long)ds3231_time, avg_roll, avg_pitch,distance*100);
            esp_mqtt_client_publish(client, "/LIPL/sensordata/sensors", msg, 0, 0, 0);
            printf("Time: %lld   |   Roll: %.2f  |  Pitch: %.2f   |   Distance:%0.02f \n", (long long)ds3231_time, avg_roll, avg_pitch,distance*100);
            global_pitch = 0;
            global_roll = 0;
        }
        // vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
    }
    
}


void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    while (true)
    {
        
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                     vTaskDelay(pdMS_TO_TICKS(1000));
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    vTaskDelay(pdMS_TO_TICKS(2000));
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                     vTaskDelay(pdMS_TO_TICKS(500)); 
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else
          // printf("Distance: %0.04f cm\n", distance*100);

        vTaskDelay(pdMS_TO_TICKS(500));
        
    }
}

// Function to test the MPU6050 sensor
void test_mpu6050()
{
    uint8_t MPU_ID;
    if (mpu6050_get_deviceid(sensor, &MPU_ID) != ESP_OK)
    {
        ESP_LOGE(TAG, "MPU6050 Failure detected.");
        MPU6050_FAIL = true;
    }
    else
    {
        ESP_LOGI(TAG, "MPU6050 device ID: %d", MPU_ID);
        MPU6050_FAIL = false;
    }

    // If sensor fails, try to wake it up or reinitialize
  
    if (MPU6050_FAIL)
    {
        ESP_LOGW(TAG, "Attempting to restart MPU6050...");
        if (mpu6050_wake_up(sensor) != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to wake up MPU6050.");
        }
        else
        {
            ESP_LOGI(TAG, "MPU6050 restart successful.");
            MPU6050_FAIL = false;
        }
    }
}


//git test

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init();
    i2c_EspMpu_init(I2C_PORT, 400000);
    init_ds3231(I2C_PORT_RTC, 100000);

    vTaskDelay(pdMS_TO_TICKS(5000));    

    while (1)
    {

        vTaskDelay(pdMS_TO_TICKS(100));
        test_mpu6050();
        
        if (wifi_conn_flag == true)
        {
            break;
        }
    }

    // Set up the timer for MPU6050
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .arg = NULL,
        .name = "mpu6050_timer"};

    esp_timer_handle_t timer_handle;
    esp_err_t ret = esp_timer_create(&timer_args, &timer_handle);
    if (ret != ESP_OK)
    {
        printf("Failed to create timer\n");
        return;
    }

    // Start the timer, set to trigger every 2.5ms
    ret = esp_timer_start_periodic(timer_handle, 2500); // Adjusted for ms to µs
    if (ret != ESP_OK)
    {
        printf("Failed to start timer\n");
        return;
    }

    // Start the MQTT client
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&(esp_mqtt_client_config_t){
        .broker.address.uri = CONFIG_BROKER_URL,
    });
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);

    // Create the sensor data reading task
    xTaskCreate(&mpu6050_task, "mpu6050_task", 4096, NULL, 5, NULL);
    xTaskCreate(&mqtt_publish_task, "mqtt_publish_task", 65536, client, 5, NULL);
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
    // vTaskDelay((DISPLAY_INTERVAL_MS - SAMPLE_INTERVAL_MS * count) / portTICK_PERIOD_MS);
    global_pitch = 0;
    global_roll = 0;
    vTaskDelay((DISPLAY_INTERVAL_MS - SAMPLE_INTERVAL_MS * count) / portTICK_PERIOD_MS);
}


//66b5e59a388d38768bb00a0d: Sim1 
//66b5e59f388d38768bb00a0e: Sim2
//66b5e5b0388d38768bb00a0f: Sim3
