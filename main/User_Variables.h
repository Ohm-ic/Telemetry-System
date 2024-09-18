#ifndef USER_VARIABLES_H
#define USER_VARIABLES_H
#include "esp_littlefs.h"

//WiFi Provisioning Mgr Variables

const int WIFI_CONNECTED_EVENT = BIT0;
const int WIFI_PROV_TIMEOUT_EVENT = BIT1;
const int WIFI_PROV_DONE = BIT2;

bool provisioned = false;
bool wificonnected = false;
// MQTT connection status flag
bool mqtt_connect_status_flag = false;


// LittleFS  Variables 

//Lfs config
esp_vfs_littlefs_conf_t configLfs_Conf = {
        .base_path = "/littlefs",      //Mounting point name
        .partition_label = "littlefs", // Label as on partitions.csv
        .format_if_mount_failed= true, //Format if failed 
        .dont_mount = false,           //Dont attempt to mount or format? overrides format_if_failed
};
//Sensor File name
char *sensorfile = "/littelfs/sensor.txt";

//FREERTOS handles 

//Task handles 

TaskHandle_t Sensor_task_handle_t;
// Event Group Handles

//Wifi event group handle 
EventGroupHandle_t wifi_event_group;

//Queue Handles 


//Mutex Handles
SemaphoreHandle_t SensorfileMutexHandle;
static portMUX_TYPE my_mutex;

//Simple NTP configs
#define USER_CONFIG_SNTP_SYNC_INTERVAL 15*1000        //Sync freq in milliseconds cannot be lower than 15s
#define CONFIG_TIME_ZONE               5.5            //+5hrs and 30 mins is Indian Standard Time (5.5) set to 0 for UTC (default)

// MQTT handles
esp_mqtt_client_config_t mqtt_cfg;
esp_mqtt_client_handle_t client;

//POLLING PERIODS
#define RESET_BUTTON_POLL_PERIOD_MS 100
int global_polling_rate = 1; // Polling rate in seconds
int ultrasonic_ping_times = 70;
int ultrasonic_ping_rate_ms  = 50; //Ping rate in mS
//Global I2C device handles
i2c_dev_t i2c_ds3231;
i2c_dev_t i2c_mpu6050;
 
//Global sensor handles 
static mpu6050_handle_t mpu6050_dev = NULL;

//Global sensor values
static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;
long   rtc_time;

// Mac Address
u8_t mac_addr[6];
char mac[20];

#endif 