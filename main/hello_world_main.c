#include <stdio.h>

#include <driver/gpio.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <driver/rtc_io.h>
#include <driver/spi_master.h>
#include <driver/ledc.h>
#include <driver/i2c.h>
//#include <driver/rtc_cntl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_netif.h>
#include <esp_timer.h>
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
#include <mqtt_client.h>

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

static int s_retry_num = 0;

static const char* TAG = "TEST";

static const proto8_t self_tag[] = {1};

struct _power_data_t
{
    unsigned char data[3];
} typedef power_data_t;

struct _light_data_t
{
    unsigned char data[5];
} typedef light_data_t;

struct _move_data_t
{
    unsigned char data[15];
} typedef move_data_t;

power_data_t power_data[1];
unsigned char power_index[1] = {0};
light_data_t light_data[1];
unsigned char light_index[1] = {0};
move_data_t move_data[1];
unsigned char move_index[1] = {0};

esp_mqtt_client_handle_t client;

#define CONFIG_BROKER_URL "mqtt://54.157.172.217"

#define SELF_TAG_SIZE sizeof(self_tag)
//#define SERVER_TAG_SIZE sizeof(sector_tag)

#define PIR_UPLOAD_SIZE 15
#define LDR_UPLOAD_SIZE 5
#define PWR_UPLOAD_SIZE 3

#define PIR_TEMPLATE_HEADER "PATCH /entities/%u/%u/presence HTTP/1.0\r\nContent-Type: application/json\r\nContent-Length:%u\r\n\r\n%s"
#define PIR_TEMPLATE_DATA "{\"timestamp\":%lu,\"values\":\"[%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu,%hhu]\"}"
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
#define ADS1115_CONFH_A0 0x42
#define ADS1115_CONFH_A1 0x52
#define ADS1115_CONFL 0x00 //0xE0
#define ADS1115_TH 0xFF
#define ADS1115_TL 0x00

#define ADS1015_ADDR 0x49
#define ADS1015_PTR_DATA ADS1115_PTR_DATA
#define ADS1015_PTR_CONF ADS1115_PTR_CONF
#define ADS1015_PTR_THRESHL ADS1115_PTR_THRESHL
#define ADS1015_PTR_THRESHH ADS1115_PTR_THRESHH
#define ADS1015_CONFH_A0 ADS1115_CONFH_A0
#define ADS1015_CONFH_A1 ADS1115_CONFH_A1
#define ADS1015_CONFL 0x00 //0xE0
#define ADS1015_TH ADS1115_TH
#define ADS1015_TL ADS1115_TL

#define FALSE 0
#define TRUE 1

#define GPIO_ZEROCROSS_PIN 22
#define GPIO_TRIAC_PIN 5
#define GPIO_INTR_VOLT_PIN 4
#define GPIO_INTR_CURR_PIN 2
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_TRIAC_PIN)
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_ZEROCROSS_PIN)
#define GPIO_INTR_PIN_SEL (1ULL << GPIO_INTR_VOLT_PIN | 1ULL << GPIO_INTR_CURR_PIN)

#define VOLT_DC_OFFSET X
#define CURR_DC_OFFSET X

uint8_t duty_min = 0, duty_max = 0, duty = 0, volt_has_reading = FALSE, curr_has_reading = FALSE;

int tag_to_index(proto8_t tag[], proto8_t size)
{
    switch (size)
    {
    case 1:
        switch (tag[0])
        {
        case 2:
            return 1;
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

void tcp_communicate(char payload[])
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
        return;
    }
    int err = connect(sock, (struct sockaddr*) &addr, sizeof(struct sockaddr_in));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Connect error: %d", err);
        return;
    }
    err = send(sock, payload, strlen(payload), 0);
    if (err < 0)
    {
        ESP_LOGE(TAG, "Send error: %d", errno);
        return;
    }
    char recv_buffer[128];
    int len = recv(sock, recv_buffer, sizeof(recv_buffer)-1, 0);
    if (len < 0)
    {
        ESP_LOGE(TAG, "Recv error: %d", errno);
        return;
    }
    printf(recv_buffer);
    if (sock != -1)
    {
        ESP_LOGI(TAG, "Transmission Concluded.");
        shutdown(sock, 0);
        close(sock);
    }
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
    sprintf(payload_buffer, PIR_TEMPLATE_HEADER, tag_to_index((proto8_t*) self_tag, SELF_TAG_SIZE), sensor, strlen(data_buffer), data_buffer);
    tcp_communicate(payload_buffer);
    printf("DONE\n");
}

void store_pir(proto8_t payload[], proto16_t length)
{
    for (int i = 0; i < length; i++)
    {
        move_data[0].data[move_index[0]] = payload[i];
        move_index[0]++;
        if (move_index[0] == PIR_UPLOAD_SIZE)
        {
            move_index[0] = 0;
            upload_pir(0);
        }
    }
}

void store_ldr(proto8_t payload[], proto16_t length)
{
    return;
    light_data[0].data[light_index[0]] = payload[0];
    light_index[0]++;
}

void message_broker(proto_data_unit_t* pdu)
{
    if (compare_header(pdu->header, head_send_pir))
    {
        store_pir(pdu->payload, pdu->length);
    }
    else if (compare_header(pdu->header, head_send_ldr))
    {
        store_ldr(pdu->payload, pdu->length);
    }
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

void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id_, void* event_data)
{
    printf("base: %s, event_id: %d", base, event_id_);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        esp_mqtt_client_publish(client, "/iot2/motion2", "5", 0, 0, 0);
        kkek = 1;
        printf("KEK ZAP\n");
        break;
    case MQTT_EVENT_PUBLISHED:
        printf("msg_id: %d\n", event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        printf("ERROR MQTT\n");
        break;
    default:
        printf("UKN ERROR\n");
        break;
    }
}

void mqtt_start()
{
    esp_mqtt_client_config_t mqtt_cfg = {.uri = CONFIG_BROKER_URL};
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
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

    return err;
}

esp_err_t read_i2c_ads1115(uint16_t* data)
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

esp_err_t read_i2c_ads1015(uint16_t* data)
{
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
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
    
    cmd = i2c_cmd_link_create();
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

/*uint16_t read_i2c_power_sensor()
{
    uint16_t voltage, current, power;
    read_i2c_ads1115(&voltage, ADS1015_CONFH_A0);
    read_i2c_ads1115(&current, ADS1015_CONFH_A1);
    power = voltage * current;
    
    return power;
}*/

void task_dimmer(void* param)
{
    struct timeval now, ref;
    gettimeofday(&ref, NULL);
    uint8_t zerocross_on = FALSE;
    for (;;)
    {
        int zerocross = gpio_get_level(GPIO_ZEROCROSS_PIN);
        if(zerocross && !zerocross_on)
        {
            gettimeofday(&ref, NULL);
            zerocross_on = TRUE;
            gpio_set_level(GPIO_TRIAC_PIN, FALSE);
            if(1/*MAX*/ && duty != duty_max)
            {
                duty++;
            }
            else if (!1/*MAX*/ && duty != duty_min)
            {
                duty--;
            }
        }
        else if (!zerocross)
            zerocross_on = FALSE;
        gettimeofday(&now, NULL);
        uint32_t diff = now.tv_usec - ref.tv_usec;
        if (diff & 0x80000000)
            diff += 1000000;
        if (diff >= (uint16_t) (duty * 83 + (duty * 2 + 2)/6))
        {
            gpio_set_level(GPIO_TRIAC_PIN, TRUE);
        }
    }
    
}

static void IRAM_ATTR intr_volt_handler(void* arg)
{
    volt_has_reading = TRUE;
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

void task_volt_reading(void* param)
{
    if (volt_has_reading)
    {
        uint16_t data;
        read_i2c_ads1115(&data);
        printf("VOLT: %hu\n", data);

    }
}

void task_curr_reading(void* param);

void app_main()
{
    setup_gpio();
    //esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    xTaskCreate(task_dimmer, "dimmer", 4096, NULL, 10, NULL);
    xTaskCreate(task_volt_reading, "volt", 4096, NULL, 20, NULL);
    //xTaskCreate(task_curr_reading, "volt", 4096, NULL, 30, NULL);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_sta();
    //xTaskCreate(udp_server_task, "udp_server", 4096, NULL, 5, NULL);

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "time1.google.com");
    sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    sntp_init();
    time_t now;
    time(&now);
    printf("Time: %lu\n", now);
    
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
    //mqtt_start();
    
    i2c_master_init();
    setup_i2c_ads1115();
    setup_i2c_ads1015();
    //uint16_t test;
    for (;;)
    {
        time_t now;
        time(&now);
        printf("Time: %lu\n", now);
        vTaskDelay(500);
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
