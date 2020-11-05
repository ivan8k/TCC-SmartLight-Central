#include <stdio.h>
#include <math.h>

#include <driver/gpio.h>
//#include <driver/adc.h>
//#include <driver/dac.h>
#include <driver/rtc_io.h>
//#include <driver/spi_master.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
#include <driver/timer.h>
//#include <driver/rtc_cntl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
//#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_netif.h>
//#include <esp_timer.h>
#include <esp_intr_alloc.h>
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/apps/sntp.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <soc/sens_reg.h>
#include <soc/rtc_periph.h>
#include <soc/rtc.h>
#include <time.h>
#include <sys/time.h>

#include <lwip/err.h>
#include <lwip/sys.h>

#include <sdkconfig.h>

#include "protocol.c"

char kkek = 0;

//#define HOST_IP "54.157.172.217"
#define HOST_IP "192.168.15.10"
//#define PORT 1026
#define PORT 40404
#define CLOUD_IP "54.157.172.217"
#define CLOUD_PORT 6042

#define SSID      CONFIG_ESP_WIFI_SSID
#define PASS      CONFIG_ESP_WIFI_PASS
#define MAX_RETRY CONFIG_ESP_MAXIMUM_RETRY

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static int s_retry_num = 0;

static const char* TAG = "TEST";

static const proto8_t self_tag[] = {1};

struct _power_data_t
{
    double data[3];
} typedef power_data_t;

//typedef double power_data_t;

struct _light_data_t
{
    uint16_t data[5];
} typedef light_data_t;

struct _move_data_t
{
    uint8_t data[15];
} typedef move_data_t;

#define SENSOR_NUM 2

power_data_t power_data;
//power_data_t power_data;
unsigned char power_index = 0;
light_data_t light_data[SENSOR_NUM];
unsigned char light_index[SENSOR_NUM] = {0, 0};
move_data_t move_data[SENSOR_NUM];
unsigned char move_index[SENSOR_NUM] = {0, 0};

#define SELF_TAG_SIZE sizeof(self_tag)
//#define SERVER_TAG_SIZE sizeof(sector_tag)

#define PIR_UPLOAD_SIZE 15
#define LDR_UPLOAD_SIZE 5
#define PWR_UPLOAD_SIZE 3
#define PWR_STORE_TIME_S 180

#define ENTITY_ID "proto"

#define PIR_TEMPLATE_HEADER "PATCH /entities/%s/%u/%u/presence HTTP/1.0\r\nContent-Type: application/json\r\nContent-Length:%u\r\n\r\n%s"
#define PIR_TEMPLATE_DATA "{\"timestamp\":%lu,\"values\":\"[%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu]\"}"
#define LDR_TEMPLATE_HEADER "PATCH /entities/%s/%u/%u/light HTTP/1.0\r\nContent-Type: application/json\r\nContent-Length:%u\r\n\r\n%s"
#define LDR_TEMPLATE_DATA "{\"timestamp\":%lu,\"values\":\"[%hu,%hu,%hu,%hu,%hu]\"}"
#define PWR_TEMPLATE_HEADER "PATCH /entities/%s/%u/power HTTP/1.0\r\nContent-Type: application/json\r\nContent-Length:%u\r\n\r\n%s"
#define PWR_TEMPLATE_DATA "{\"timestamp\":%lu,\"values\":\"[%lf,%lf,%lf]\"}"
#define POLL_TEMPLATE "GET /entities/%s/%u/poll HTTP/1.0\r\n\r\n"
#define STATUS_TEMPLATE_HEADER "PUT /entities/%s/status/%u/%u HTTP/1.0\r\nContent-Type: application/json\r\nContent-Length:%u\r\n\r\n%s"
#define STATUS_TEMPLATE_DATA "{\"code\":%hhu}"
    //                                ""

//#define PIR_PDU_SIZE(X) (PIR_HEADER_SIZE+SELF_TAG_SIZE+SERVER_TAG_SIZE+2+PIR_PAYLOAD_SIZE(X))
//#define LDR_PDU_SIZE(X) (LDR_HEADER_SIZE+SELF_TAG_SIZE+SERVER_TAG_SIZE+2+LDR_PAYLOAD_SIZE(X))

#define I2C_SDA_GPIO_NUM 18
#define I2C_SCL_GPIO_NUM 19
#define I2C_CLK_SPEED 350000
#define I2C_PORT I2C_NUM_0
#define I2C_ACK_CHECK_EN 0x1
#define I2C_ACK_CHECK_DIS 0x0
#define I2C_ACK 0x0
#define I2C_NACK 0x1

#define ADS1115_ADDR 0x48
#define ADS1115_PTR_DATA 0x0
#define ADS1115_PTR_CONF 0x1
#define ADS1115_PTR_THRESHL 0x2
#define ADS1115_PTR_THRESHH 0x3
//#define ADS1115_CONF 0x2
#define ADS1115_CONFH_A0 0x02//0x42
#define ADS1115_CONFH_A1 0x52
#define ADS1115_CONFL 0xE8 //0xE0
#define ADS1115_TH 0xFF
#define ADS1115_TL 0x00

#define ADS1015_ADDR 0x49
#define ADS1015_PTR_DATA ADS1115_PTR_DATA
#define ADS1015_PTR_CONF ADS1115_PTR_CONF
#define ADS1015_PTR_THRESHL ADS1115_PTR_THRESHL
#define ADS1015_PTR_THRESHH ADS1115_PTR_THRESHH
#define ADS1015_CONFH_A0 ADS1115_CONFH_A0
#define ADS1015_CONFH_A1 ADS1115_CONFH_A1
#define ADS1015_CONFL 0xE0 //ADS1115_CONFL //0x00 //0xE0
#define ADS1015_TH ADS1115_TH
#define ADS1015_TL ADS1115_TL

#define FALSE 0
#define TRUE 1

#define GPIO_ZEROCROSS_PIN 25//22
#define GPIO_TRIAC_PIN 5
#define GPIO_INTR_VOLT_PIN 4
#define GPIO_INTR_CURR_PIN 2
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_TRIAC_PIN)
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_ZEROCROSS_PIN)
#define GPIO_INTR_PIN_SEL (1ULL << GPIO_INTR_VOLT_PIN | 1ULL << GPIO_INTR_CURR_PIN)

#define VOLT_DC_OFFSET 13000//13055
#define VOLT_FACTOR 116.62 //118.2
#define CURR_DC_OFFSET 13000
#define CURR_FACTOR 6.54//30.0

#define PWR_SAMPLE_TIME_S 30 // 60 * 5

#define TIMER_DIVIDER 16
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)
#define DIMMER_DELAY_SEC 10

#define WATCHDOG_TIME_S 30
#define WATCHDOG_SCAN_TIME_MS 5000

uint8_t duty_min = 0, duty_max = 0, duty = 50, volt_has_reading = FALSE, curr_has_reading = FALSE;
double v_rms = 0.0;
const float root2 = 1.41421356;

time_t energy_time;
double energy_data;
uint8_t energy_ready = FALSE;

uint8_t is_max = FALSE;

time_t watchdog[SENSOR_NUM];
uint8_t status[SENSOR_NUM+1];
uint8_t status_changed[SENSOR_NUM+1];

int tag_to_index(proto8_t tag[], proto8_t size)
{
    switch (size)
    {
    case 1:
        switch (tag[0])
        {
        case 1:
            return 0;
        case 2:
            return 1;
        case 3:
            return 2;
        default:
            return -1;
        }
    default:
        return -1;
    }
}

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();
            ESP_LOGI(TAG, "Connecting...");
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            if (s_retry_num >= MAX_RETRY)
            {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGI(TAG, "Max retry reached.");
                return;
            }
            else
            {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "Retrying...");
            }
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        //ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Connected");
        //connected
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = 
    {
        .sta = 
        {
            .ssid = SSID,
            .password = PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Conectado a: %s", SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Erro na connexão wifi");
    }
    else
    {
        ESP_LOGW(TAG, "Evento desconhecido");
    }
    
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_got_ip));
    vEventGroupDelete(s_wifi_event_group);
}

int tcp_communicate(char payload[], char recv_buffer[], int recv_size)
{
    //static const char* test_payload = "POST /v2/entities HTTP/1.0\r\nContent-Type: application/json\r\nContent-Length:28\r\n\r\n"
    //                                "{\"id\":\"Test2\",\"type\":\"Test\"}";
    //static const char* test_payload = "test package";
    //char test_payload[50];
    //sprintf(test_payload, "test package #%i", ulp_pir_sensor_readings & UINT16_MAX);
    //ESP_LOGI(test_payload, "");
    printf("||| TRYING TO CONNECT |||\n");
    //printf(test_payload);
    //printf("\n");
    
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in addr;
    addr.sin_addr.s_addr = inet_addr(CLOUD_IP);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(CLOUD_PORT);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Socket error: %d", errno);
        return -1;
    }
    printf("SOCK\n");
    int err = connect(sock, (struct sockaddr*) &addr, sizeof(struct sockaddr_in));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Connect error: %d", err);
        return -1;
    }
    printf("CONN\n");
    err = send(sock, payload, strlen(payload), 0);
    if (err < 0)
    {
        ESP_LOGE(TAG, "Send error: %d", errno);
        return -1;
    }
    printf("SEND\n");
    int len = 0;
    //char recv_buffer[128];
    if (recv_buffer != NULL)
    {
        len = recv(sock, recv_buffer, recv_size-1, 0);
        if (len < 0)
        {
            ESP_LOGE(TAG, "Recv error: %d", errno);
            return -1;
        }
        printf("%s\n",recv_buffer);
    }
    printf("RECV\n");
    if (sock != -1)
    {
        ESP_LOGI(TAG, "Transmission Concluded.");
        shutdown(sock, 0);
        close(sock);
    }
    printf("CLOSE\n");
    return len;
}

void upload_pir(unsigned short sensor)
{
    char payload_buffer[512];
    char data_buffer[256];
    time_t now;
    time(&now);
    sprintf(data_buffer, PIR_TEMPLATE_DATA, now, move_data[sensor].data[sensor], move_data[sensor].data[1], move_data[sensor].data[2],
    move_data[sensor].data[3], move_data[sensor].data[4], move_data[sensor].data[5], move_data[sensor].data[6], move_data[sensor].data[7],
    move_data[sensor].data[8], move_data[sensor].data[9], move_data[sensor].data[10], move_data[sensor].data[11],
    move_data[sensor].data[12], move_data[sensor].data[13], move_data[sensor].data[14]);
    sprintf(payload_buffer, PIR_TEMPLATE_HEADER, ENTITY_ID, tag_to_index((proto8_t*) self_tag, SELF_TAG_SIZE)+1, sensor+1, strlen(data_buffer), data_buffer);
    tcp_communicate(payload_buffer, NULL, 0);
    //printf("DONE\n");
}

void upload_ldr(unsigned short sensor)
{
    return;
    char payload_buffer[512];
    char data_buffer[256];
    time_t now;
    time(&now);
    sprintf(data_buffer, PIR_TEMPLATE_DATA, now, move_data[sensor].data[sensor], move_data[sensor].data[1], move_data[sensor].data[2],
    move_data[sensor].data[3], move_data[sensor].data[4], move_data[sensor].data[5], move_data[sensor].data[6], move_data[sensor].data[7],
    move_data[sensor].data[8], move_data[sensor].data[9], move_data[sensor].data[10], move_data[sensor].data[11],
    move_data[sensor].data[12], move_data[sensor].data[13], move_data[sensor].data[14]);
    sprintf(payload_buffer, PIR_TEMPLATE_HEADER, ENTITY_ID, tag_to_index((proto8_t*) self_tag, SELF_TAG_SIZE)+1, sensor+1, strlen(data_buffer), data_buffer);
    tcp_communicate(payload_buffer, NULL, 0);
    //printf("DONE\n");
}

void upload_pwr()
{
    char payload_buffer[256];
    char data_buffer[128];
    sprintf(data_buffer, PWR_TEMPLATE_DATA, energy_time, power_data.data[0], power_data.data[1], power_data.data[2]);
    sprintf(payload_buffer, PWR_TEMPLATE_HEADER, ENTITY_ID, tag_to_index((proto8_t*) self_tag, SELF_TAG_SIZE)+1, strlen(data_buffer), data_buffer);
    tcp_communicate(payload_buffer, NULL, 0);

}

void upload_status(unsigned short sensor, uint8_t code)
{
    char payload_buffer[256];
    char data_buffer[128];
    sprintf(data_buffer, STATUS_TEMPLATE_DATA, code);
    sprintf(payload_buffer, STATUS_TEMPLATE_HEADER, ENTITY_ID, tag_to_index((proto8_t*) self_tag, SELF_TAG_SIZE)+1, sensor+1, strlen(data_buffer), data_buffer);
    tcp_communicate(payload_buffer, NULL, 0);
}

void store_pir(proto8_t payload[], proto16_t length, int idx)
{
    for (int i = 0; i < length; i++)
    {
        move_data[idx].data[move_index[idx]] = payload[i];
        move_index[idx]++;
        if (move_index[idx] == PIR_UPLOAD_SIZE)
        {
            move_index[idx] = 0;
            upload_pir(idx);
        }
    }
}

void store_ldr(proto8_t payload[], proto16_t length, int idx)
{
    for (int i = 0; i < length; i++)
    {
        light_data[idx].data[light_index[idx]] = payload[i];
        light_index[idx]++;
        if (light_index[idx] == LDR_UPLOAD_SIZE)
        {
            light_index[idx] = 0;
            upload_ldr(idx);
        }
    }
    light_data[0].data[light_index[0]] = payload[0];
    light_index[0]++;
}

void message_broker(proto_data_unit_t* pdu)
{
    int s_tag = tag_to_index(pdu->sensor_tag, pdu->sensor_tag_size);
    if (compare_header(pdu->header, head_send_pir))
    {
        store_pir(pdu->payload, pdu->length, s_tag);
    }
    else if (compare_header(pdu->header, head_move_trigger))
    {
        timer_start(TIMER_GROUP_0, TIMER_0);
        is_max = TRUE;
    }
    else if (compare_header(pdu->header, head_send_ldr))
    {
        store_ldr(pdu->payload, pdu->length, s_tag);
        if (status[s_tag] != 0)
        {
            status[s_tag] = 0;
            status_changed[s_tag] = TRUE;
        }
    }
    else if (compare_header(pdu->header, head_sensor_error))
    {
        if (status[s_tag] != 2)
        {
            status[s_tag] = 2;
            status_changed[s_tag] = TRUE;
        }
    }
    time_t now;
    time(&now);
    watchdog[tag_to_index(pdu->sensor_tag, pdu->sensor_tag_size)] = now;
}

proto16_t get_response(proto_data_unit_t* pdu, proto8_t response[])
{
    proto16_t response_size = make_ok_data_unit(response, (proto8_t*) pdu->sector_tag, pdu->sensor_tag);
    return response_size;
} 

proto16_t handle_message(char recv_buffer[], proto16_t recv_size, char response[])
{
    proto8_t* buffer = (proto8_t*) recv_buffer;
    proto_data_unit_t pdu = unmake_data_unit(buffer);
    message_broker(&pdu);
    proto8_t* resp = (proto8_t*) response;

    return get_response(&pdu, resp);
}

void udp_server_task(void* param)
{
    char recv_buffer[256];
    char response_buffer[256];

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Socket error: %d", errno);
        return;
    }

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    int err = bind(sock, (struct sockaddr*) &dest_addr, sizeof(struct sockaddr_in));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Socket bind error: %d", errno);
        return;
    }

    while (1)
    {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(struct sockaddr_in);
        int len = recvfrom(sock, recv_buffer, 255, 0, (struct sockaddr*) &source_addr, &socklen);
        if (len < 0)
        {
            //ESP_LOGE(TAG, "");
            break;
        }
        printf("RECEIVED\n");
        proto16_t resp_len = handle_message(recv_buffer, len, response_buffer);
        int err = sendto(sock, response_buffer, resp_len, 0, (struct sockaddr*) &source_addr, sizeof(struct sockaddr_in));
        if (err < 0)
        {
            //ESP_LOGE(TAG, "");
            break;
        }
    }
    if (sock >= 0)
    {
        shutdown(sock, 0);
        close(sock);
    }
    printf("exit||||||\n");
}

void i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_GPIO_NUM;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_SCL_GPIO_NUM;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_CLK_SPEED;
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

esp_err_t setup_i2c_ads1115()
{
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_PTR_THRESHH, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_TH, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_TH, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_PTR_THRESHL, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_TL, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_TL, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_PTR_CONF, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_CONFH_A0, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_CONFL, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1115_PTR_DATA, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

esp_err_t setup_i2c_ads1015()
{
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1015_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_PTR_THRESHH, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_TH, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_TH, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1015_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_PTR_THRESHL, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_TL, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_TL, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1015_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_PTR_CONF, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_CONFH_A0, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_CONFL, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1015_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADS1015_PTR_DATA, I2C_ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (err)
    {
        printf("ERRO CURR\n");
    }
    return err;
}

esp_err_t read_i2c_ads1115(int16_t* data)
{
    esp_err_t err;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1115_ADDR << 1) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
    uint8_t datah = 0xFF, datal=0x0;
    i2c_master_read_byte(cmd, &datah, I2C_ACK);
    i2c_master_read_byte(cmd, &datal, I2C_ACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    *data = (datah << 8) + datal;

    return err;
}

esp_err_t read_i2c_ads1015(int16_t* data)
{/*
    *data = CURR_DC_OFFSET + 1;
    return ESP_OK;*/
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADS1015_ADDR << 1) | I2C_MASTER_READ, I2C_ACK_CHECK_EN);
    uint8_t datah = 0xFF, datal=0x0;
    i2c_master_read_byte(cmd, &datah, I2C_ACK);
    i2c_master_read_byte(cmd, &datal, I2C_ACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    *data = (datah << 8) + datal;

    return err;
}

static void IRAM_ATTR intr_volt_handler(void* arg)
{
    volt_has_reading = TRUE;
}

static void IRAM_ATTR intr_curr_handler(void* arg)
{
    curr_has_reading = TRUE;
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_0);
}

void setup_gpio()
{
    gpio_config_t conf;
    conf.intr_type = GPIO_INTR_DISABLE;
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    conf.pull_down_en = FALSE;
    conf.pull_up_en = FALSE;
    gpio_config(&conf);
    
    conf.mode = GPIO_MODE_INPUT;
    conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    gpio_config(&conf);
    
    conf.intr_type = GPIO_INTR_POSEDGE;
    conf.pin_bit_mask = GPIO_INTR_PIN_SEL;
    conf.pull_up_en = TRUE;
    gpio_config(&conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_INTR_VOLT_PIN, intr_volt_handler, (void*) GPIO_INTR_VOLT_PIN);
    //gpio_isr_handler_add(GPIO_INTR_CURR_PIN, intr_curr_handler, (void*) GPIO_INTR_CURR_PIN);
}

/*VOLT: 119.729439
MIN: -4137
MAX> 4148
VOLT: 119.729439
MIN: -4115
MAX> 4188*//*
void task_volt_reading(void* param)//17249         17226 - 116.1V -- 4096 -> 118.2V
{
    for (;;) // 114.3V 119.8V    0.5V 0V
    {
        if (volt_has_reading)
        {
            volt_has_reading = FALSE;
            read_i2c_ads1115(&volt);
            //printf("READ: %hi\n", volt);
            int16_t aux_volt = volt - volt_dc_offset;
            if (aux_volt > volt_max)
            {
                volt_max = aux_volt;
            }
            else if (aux_volt < volt_min)
            {
                volt_min = aux_volt;
            }
            if (!++volt_count)
            {
                volt_max_sum += volt_max;
                volt_min_sum += volt_min;
                volt_minmax_count++;
                volt_minmax_count &= 0x7; // volt_minmax_count = (volt_minmax_count+1) % 8;
                if (!volt_minmax_count)
                {
                    int16_t aux = ((volt_max_sum / 8) - (volt_min_sum / 8)) / 2;
                    v_rms = 116.62;//118.2;
                    v_rms *= aux;
                    v_rms /= 4096;
                    
                    volt_max_sum = 0;
                    volt_min_sum = 0;

                    v_rms /= root2;
                    printf("VOLT: %f\n", v_rms);
                    //printf("VOLT: %f\nMIN: %hi\nMAX> %hi\n", v_rms, volt_min, volt_max);
                }
                volt_min = 0;
                volt_max = 0;
            }
        }
        vTaskDelay(0 / portTICK_RATE_MS);
    }
}

void task_curr_reading(void* param)
{
    struct timeval now, ref;
    do ; while (!curr_has_reading);
    curr_has_reading = FALSE;
    read_i2c_ads1015(&curr_prev);
    curr_prev -= curr_dc_offset;
    gettimeofday(&ref, NULL);
    
    for (;;)
    {
        
        if (curr_has_reading)
        {
            curr_has_reading = FALSE;
            read_i2c_ads1015(&curr);
            curr -= curr_dc_offset;
            gettimeofday(&now, NULL);
            
            uint32_t diff = now.tv_usec - ref.tv_usec;
            if (diff & 0x80000000)
                diff += 1000000;
            double aux = (double)v_rms;
            aux *= (double)(curr + curr_prev)/2.0;
            aux *= diff * 1.0 / 1000000.0;
            energy += aux;
            ref.tv_usec = now.tv_usec;
        }
        vTaskDelay(0 / portTICK_RATE_MS);
    }
}*/

uint8_t cnt = 0;
int32_t sum = 0;
int testejj = 0;

void task_i2c_read(void* param)
{
    // voltage sensor
    int16_t volt_max = 0x8000, volt_max_aux = 0x8000, volt_min = 0x7FFF, volt_min_aux = 0x7FFF;
    int32_t volt_max_sum = 0, volt_min_sum = 0;
    uint8_t volt_count = 0, volt_minmax_count = 0;
    int16_t volt, volt_dc_offset = 0;//VOLT_DC_OFFSET;

    // current sensor
    double curr, curr_prev;// = -0xFFF;
    int16_t curr_dc_offset = 0;//CURR_DC_OFFSET;
    double curr_sum = 0.0;
    uint64_t time_sum = 0;
    int16_t curr_max = 0x8000, curr_min = 0x7FFF;
    uint8_t curr_count = 0;

    struct timeval now_curr, ref_curr;
    //do ; while (!curr_has_reading);
    //curr_has_reading = FALSE;
    int16_t curr_aux;
    read_i2c_ads1015(&curr_aux);
    gettimeofday(&ref_curr, NULL);
    curr_prev = curr_aux - curr_dc_offset;
    curr_prev *= CURR_FACTOR;
    curr_prev /= 4096.0;
    curr_prev *= curr_prev;
    //curr_prev -= curr_dc_offset;

    // dimmer
    struct timeval now_dimmer, ref_dimmer, aux_dimmer;
    gettimeofday(&ref_dimmer, NULL);
    uint8_t zerocross_on = FALSE, aux_delay = FALSE;

    for (;;)
    {
        // voltage sensor reading
        if (volt_has_reading)
        {
            volt_has_reading = FALSE;
            if (!read_i2c_ads1115(&volt))
            {
                //read_i2c_ads1115(&volt);
                if (volt > volt_max_aux)
                    volt_max_aux = volt;
                else if (volt < volt_min_aux)
                    volt_min_aux = volt;
                
                volt -= volt_dc_offset;
                if (volt > volt_max)
                    volt_max = volt;
                else if (volt < volt_min)
                    volt_min = volt;
                if (!++volt_count)
                {
                    volt_max_sum += volt_max;
                    volt_min_sum += volt_min;
                    volt_minmax_count++;
                    volt_minmax_count &= 0x7; // volt_minmax_count = (volt_minmax_count+1) % 8;
                    volt_dc_offset = (volt_max_aux + volt_min_aux) / 2;
                    if (!volt_minmax_count)
                    {
                        int16_t aux = ((volt_max_sum / 8) - (volt_min_sum / 8)) / 2;
                        v_rms = VOLT_FACTOR;
                        v_rms *= aux;
                        v_rms /= 4096.0;
                        
                        volt_max_sum = 0;
                        volt_min_sum = 0;

                        v_rms /= root2;
                        //printf("VOLT: %lf\n", v_rms);
                        //printf("VOLT: %hi\nMIN: %hi\nMAX> %hi\nOFF: %hi\n", volt, volt_min_aux, volt_max_aux, volt_dc_offset);
                    }
                    volt_max = 0x8000;
                    volt_min = 0x7FFF;
                    volt_max_aux = 0x8000;
                    volt_min_aux = 0x7FFF;
                }
            }
            else
            {
                if (status[SENSOR_NUM] != 2)
                {
                    status[SENSOR_NUM] = 2;
                    status_changed[SENSOR_NUM] = TRUE;
                }
            }

        }
        
        // current sensor reading
        if (curr_has_reading)
        {
            curr_has_reading = FALSE;
            if (!read_i2c_ads1015(&curr_aux))
            {
                curr_aux >>= 4;
                //printf("CURR: %hi\n", curr_aux);
                gettimeofday(&now_curr, NULL);
                curr = curr_aux;
                if (curr_aux > curr_max)
                {
                    curr_max = curr_aux;
                }
                else if (curr_aux < curr_min)
                {
                    curr_min = curr_aux;
                }
                if (!++curr_count)
                {
                    //printf("MAX: %hi\nMIN: %hi\n", curr_max, curr_min);
                    curr_dc_offset = (curr_max + curr_min) / 2;//(curr_dc_offset + ((curr_max + curr_min) / 2)) /2;
                    curr_max = 0x8000;
                    curr_min = 0x7FFF;
                }
                curr -= curr_dc_offset;
                curr *= CURR_FACTOR;
                curr /= 4096.0;
                /*if (!curr_count)
                     printf("CURR_RAW: %lf\nOFF: %hi\n", curr, curr_dc_offset);*/
                //printf("CURR: %lf\n", curr);
                curr *= curr;
                
                uint32_t diff_curr = now_curr.tv_usec - ref_curr.tv_usec;
                if (diff_curr & 0x80000000)
                    diff_curr += 1000000;
                //printf("DIFF: %u\n", diff_curr);
                time_sum += diff_curr;
                //double aux = (double)v_rms;
                double aux = (double)(curr + curr_prev)/2.0;
                aux *= diff_curr;
                aux /= 1000000.0;
                curr_sum += aux;
                ref_curr.tv_usec = now_curr.tv_usec;
                testejj++;
                if (now_curr.tv_sec - ref_curr.tv_sec >= PWR_SAMPLE_TIME_S)
                {
                    printf("TIMESUM: %llu; %i\n", time_sum, testejj);
                    testejj = 0;
                    curr_sum /= time_sum;
                    curr_sum *= 1000000.0;
                    curr_sum = sqrt(curr_sum);
                    energy_data = curr_sum;
                    energy_data *= time_sum;
                    energy_data /= 1000000.0;
                    energy_time = now_curr.tv_sec;
                    energy_ready = TRUE;
                    ref_curr.tv_sec = now_curr.tv_sec;
                    curr_sum = 0;
                    time_sum = 0;
                }
                curr_prev = curr;
            }
            else
            {
                if (status[SENSOR_NUM] != 2)
                {
                    status[SENSOR_NUM] = 2;
                    status_changed[SENSOR_NUM] = TRUE;
                }
            }
            
        }
        //gpio_set_level(GPIO_TRIAC_PIN, TRUE);
        
        int zerocross = gpio_get_level(GPIO_ZEROCROSS_PIN);
        if(zerocross && !zerocross_on)
        {
            if (!aux_delay)
            {
                gettimeofday(&aux_dimmer, NULL);
                aux_delay = TRUE;
            }
            else
            {
                gettimeofday(&ref_dimmer, NULL);
                uint32_t diff_aux = ref_dimmer.tv_usec - aux_dimmer.tv_usec;
                if (diff_aux & 0x80000000)
                    diff_aux += 1000000;
                if (diff_aux >= 400)
                {
                    //printf("DEBG: %u\n", diff_aux);
                    aux_delay = FALSE;
                    zerocross_on = TRUE;
                    gpio_set_level(GPIO_TRIAC_PIN, FALSE);
                    /*cnt++;
                    if (!cnt)
                    {
                        if (duty)
                            duty = 0;
                        else
                        {
                            duty = 50;
                        }
                        
                    }*/
                    duty = 50;
                    if(is_max && duty != duty_max)
                    {
                        //duty++;
                    }
                    else if (!is_max && duty != duty_min)
                    {
                        //duty--;
                    }
                }
            }
        }
        else if (!zerocross)
            zerocross_on = FALSE;
        gettimeofday(&now_dimmer, NULL);
        uint32_t diff = now_dimmer.tv_usec - ref_dimmer.tv_usec;
        if (diff & 0x80000000)
            diff += 1000000;
        if (diff >= (uint16_t) (duty * 83 + (duty * 2 + 2)/6))
        {
            gpio_set_level(GPIO_TRIAC_PIN, TRUE);
        }
        vTaskDelay(0 / portTICK_RATE_MS);
        //vTaskYield();
    }
    
}
/*
void init_ulp_program()
{

    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    gpio_num_t gpio_triac_pin = GPIO_TRIAC_PIN;
    gpio_num_t gpio_zerocross_pin = GPIO_ZEROCROSS_PIN;
    int rtcio_triac_pin = rtc_io_number_get(gpio_triac_pin);
    int rtcio_zerocross_pin = rtc_io_number_get(gpio_zerocross_pin);

    ulp_triac_io_num = rtcio_triac_pin;
    ulp_zerocross_io_num = rtcio_zerocross_pin;
    ulp_duty = 0;

    ulp_debug = 0;

    rtc_gpio_init(gpio_triac_pin);
    rtc_gpio_init(gpio_zerocross_pin);
    rtc_gpio_set_direction(gpio_triac_pin, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_direction(gpio_zerocross_pin, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_triac_pin);
    rtc_gpio_pulldown_dis(gpio_zerocross_pin);
    rtc_gpio_pullup_dis(gpio_triac_pin);
    //rtc_gpio_pullup_dis(gpio_zerocross_pin);
    rtc_gpio_hold_en(gpio_triac_pin);
    rtc_gpio_hold_en(gpio_zerocross_pin);

    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging();

    //ulp_set_wakeup_period(0, 100000);

    err = ulp_run(&ulp_start - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}
*/
uint8_t convert_to_duty(uint8_t num)
{//sqrt(2-2x)           pi/4-1+sqrt(3-2x)           / pi/2
    return (uint8_t) ((100.0/M_PI)*acos(((double)num)/50.0-1.0));
}

void poll_cloud()
{
    char recv_buffer[256], payload[256];
    char* testeff = ENTITY_ID;
    sprintf(payload, POLL_TEMPLATE, testeff, 1);
    int len = tcp_communicate(payload ,recv_buffer, sizeof(recv_buffer));
    int begin, sep=-1;
    for (begin = len-1; begin >= 0; begin--)
    {
        if (recv_buffer[begin] == '|')
        {
            sep = begin+1;
            recv_buffer[begin] = '\0';
        }
        else if (recv_buffer[begin] == '\n')
        {
            begin++;
            break;
        }
    }
    char* aux_min = recv_buffer+begin;
    printf("%s\n", aux_min);
    char* aux_max = recv_buffer+sep;
    printf("%s\n", aux_max);
    int min = atoi(aux_min);
    int max = atoi(aux_max);
    duty_min = convert_to_duty((uint8_t) min);
    duty_max = convert_to_duty((uint8_t) max);

    printf("MIN: %hhu, MAX: %hhu\n", duty_min, duty_max);
    
}

void IRAM_ATTR dimmer_timer(void* param)
{
    timer_pause(TIMER_GROUP_0, TIMER_0);
    is_max = FALSE;
    //printf("TESTEEE\n");

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
}

void setup_dimmer_timer()
{
    timer_config_t conf = 
    {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &conf);
    
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, DIMMER_DELAY_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, dimmer_timer, NULL, ESP_INTR_FLAG_IRAM, NULL);
}

void setup_curr_timer()
{
    timer_config_t conf = 
    {
        .divider = 2,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    };
    timer_init(TIMER_GROUP_1, TIMER_0, &conf);
    
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, TIMER_BASE_CLK / (2*3300));
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);
    timer_isr_register(TIMER_GROUP_1, TIMER_0, intr_curr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_1, TIMER_0);
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_sta();
    //xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "time1.google.com");
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_init();
    //time_t now;
    //time(&now);
    //printf("Time: %lu\n", now);
    
    setup_dimmer_timer();
    //timer_start(TIMER_GROUP_0, TIMER_0);
    //printf("TESTE\n");
    
    /*ledc_timer_config_t ledc_config = 
    {
        .duty_resolution = LEDC_TIMER_15_BIT,
        .freq_hz = 60,//1920,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_config));
    ledc_channel_config_t ledc_channel =
    {
        .channel = LEDC_CHANNEL_0,
        .duty = 0x0,//0x3FFF,
        .gpio_num = 5,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    ESP_ERROR_CHECK(ledc_set_fade_with_step(ledc_channel.speed_mode, ledc_channel.channel, 0x7FFF, 1, 10));
    ESP_ERROR_CHECK(ledc_fade_start(ledc_channel.speed_mode, ledc_channel.channel, LEDC_FADE_NO_WAIT));*/
    
    setup_gpio();
    i2c_master_init();
    setup_i2c_ads1115();
    vTaskDelay(100 / portTICK_RATE_MS);
    setup_i2c_ads1015();
    do 
    {
        vTaskDelay(100 / portTICK_RATE_MS);
    } while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET);
    //xTaskCreate(task_dimmer, "dimmer", 4096, NULL, 5, NULL);
    //xTaskCreate(task_volt_reading, "volt", 4096, NULL, 2, NULL);
    //xTaskCreate(task_curr_reading, "curr", 4096, NULL, 2, NULL);
    setup_curr_timer();
    xTaskCreate(task_i2c_read, "i2c", 4096, NULL, 20, NULL);
    is_max = TRUE;
    time_t now;
    time(&now);
    for (uint16_t i = 0; i < SENSOR_NUM; i++)
    {
        watchdog[i] = now;
        status[i] = 0;
        status_changed[i] = FALSE;
    }
    
    for (;;)
    {
        //time_t now;
        //time(&now);
        //printf("Time: %lu\n", now);
        //printf("VOLT: %f\n", v_rms);
        //if (sum_count)
        //duty_min++;
        //printf("EN: %lf\n", energy);
        //upload_pir(0);
        //poll_cloud();
        //printf("DEBUG\n");
        //time(&now);
        //printf("Time: %lu\n", now);
        //timer_start(TIMER_GROUP_0, TIMER_0);
        //printf("TEST: %i\n", testejj);
        if (energy_ready)
        {
            energy_ready = FALSE;
            power_data.data[power_index] = energy_data;
            printf("ENERGY: %lf\n", energy_data);
            if (++power_index == PWR_UPLOAD_SIZE)
            {
                power_index = 0;
                //upload_pwr();
            }
        }
        for (uint16_t i = 0; i < SENSOR_NUM; i++)
        {
            if (now - watchdog[i] >= WATCHDOG_TIME_S)
            {
                if (status[i] != 1)
                {
                    status[i] = 1;
                    status_changed[i] = TRUE;
                }
                //printf("WD @%hu\n", i);
            }
            if (status_changed[i])
            {
                status_changed[i] = FALSE;
                //upload_status(i, status[i]);
            }
        }
        if (status_changed[SENSOR_NUM])
        {
            status_changed[SENSOR_NUM] = FALSE;
            ;
        }
        vTaskDelay(WATCHDOG_SCAN_TIME_MS / portTICK_RATE_MS);

        //test = read_i2c_power_sensor();
        //printf("||%hu<<||\n", test);
        //if (kkek)
        //    esp_mqtt_client_publish(client, "/iot2/motion2", "5", 0, 0, 0);
    }
    
    /*
    dac_cw_config_t cw;
    cw.en_ch = DAC_CHANNEL_1;
    cw.scale = DAC_CW_SCALE_1;
    cw.phase = DAC_CW_PHASE_0;
    cw.freq = 240;
    cw.offset=0;

    dac_output_enable(DAC_CHANNEL_1);
    dac_cw_generator_config(&cw);
    dac_cw_generator_enable();*/

    /*
    if (cause != ESP_SLEEP_WAKEUP_ULP && cause != ESP_SLEEP_WAKEUP_TIMER)
    {
        ESP_LOGI(TAG, "Initializing ULP.");
        init_ulp_program();
    }
    else
    {
        ESP_LOGI(TAG, "Waking up, trying to send package.");
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_init());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        wifi_init_sta();
        wake_up_routine();
        esp_wifi_stop();
    }
    ESP_LOGI(TAG, "Entering deep sleep.");*/
}
