#include "pico/stdlib.h"            // Biblioteca da Raspberry Pi Pico para funções padrão (GPIO, temporização, etc.)
#include "pico/cyw43_arch.h"        // Biblioteca para arquitetura Wi-Fi da Pico com CYW43
#include "pico/unique_id.h"         // Biblioteca com recursos para trabalhar com os pinos GPIO do Raspberry Pi Pico

#include "hardware/gpio.h"          // Biblioteca de hardware de GPIO
#include "hardware/irq.h"           // Biblioteca de hardware de interrupções
#include "hardware/adc.h"           // Biblioteca de hardware para conversão ADC
#include "hardware/pwm.h"

#include "lwip/apps/mqtt.h"         // Biblioteca LWIP MQTT -  fornece funções e recursos para conexão MQTT
#include "lwip/apps/mqtt_priv.h"    // Biblioteca que fornece funções e recursos para Geração de Conexões
#include "lwip/dns.h"               // Biblioteca que fornece funções e recursos suporte DNS:
#include "lwip/altcp_tls.h"         // Biblioteca que fornece funções e recursos para conexões seguras usando TLS:

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "RVpio.pio.h"

#define LED_R 13
#define LED_G 11
#define LED_B 12
#define BUZZ_A 10
#define BUZZ_B 21
#define JOY_X 26
#define JOY_Y 27

#define WIFI_SSID "TEMPLATE"                  // Substitua pelo nome da sua rede Wi-Fi
#define WIFI_PASSWORD "TEMPLATE"      // Substitua pela senha da sua rede Wi-Fi
#define MQTT_SERVER "TEMPLATE"                // Substitua pelo endereço do host - broket MQTT: Ex: 192.168.1.107
#define MQTT_USERNAME "TEMPLATE"     // Substitua pelo nome da host MQTT - Username
#define MQTT_PASSWORD "TEMPLATE"     // Substitua pelo Password da host MQTT - credencial de acesso - caso exista

// Definição da escala de temperatura
#ifndef TEMPERATURE_UNITS
#define TEMPERATURE_UNITS 'C' // Set to 'F' for Fahrenheit
#endif

#ifndef MQTT_SERVER
#error Need to define MQTT_SERVER
#endif

// This file includes your client certificate for client server authentication
#ifdef MQTT_CERT_INC
#include MQTT_CERT_INC
#endif

#ifndef MQTT_TOPIC_LEN
#define MQTT_TOPIC_LEN 100
#endif

#define PWM_WRAP 2000
#define PWM_CLKDIV 125

//Dados do cliente MQTT
typedef struct {
    mqtt_client_t* mqtt_client_inst;
    struct mqtt_connect_client_info_t mqtt_client_info;
    char data[MQTT_OUTPUT_RINGBUF_SIZE];
    char topic[MQTT_TOPIC_LEN];
    uint32_t len;
    ip_addr_t mqtt_server_address;
    bool connect_done;
    int subscribe_count;
    bool stop_client;
} MQTT_CLIENT_DATA_T;

//struct para armazenar a pio
typedef struct pio_refs{
    PIO address;
    int state_machine;
    int offset;
    int pin;
} pio_ref;

//struct para armazenar a cor para o desenho
typedef struct rgb{
    double red;
    double green;
    double blue;
} rgb;

//struct para armazenar o desenho
typedef struct drawing {
    double figure[25]; /**< Matriz de dados da figura. */
    rgb main_color;  /**< Cor principal da figura. */
} sketch;

//definição de pio estática para manipulação facilitada através das requisições
static pio_ref my_pio;

//Configura a pio
void config_pio(pio_ref *pio);

//retorna a cor da matriz de leds
uint32_t rgb_matrix(rgb color);

//desenha na matrix de leds
void draw(sketch sketch, uint32_t led_cfg, pio_ref pio, const uint8_t vector_size);

void led_pwm();
void buzzer_pwm();

#ifndef DEBUG_printf
#ifndef NDEBUG
#define DEBUG_printf printf
#else
#define DEBUG_printf(...)
#endif
#endif

#ifndef INFO_printf
#define INFO_printf printf
#endif

#ifndef ERROR_printf
#define ERROR_printf printf
#endif

// Temporização da coleta de temperatura - how often to measure our temperature
#define TEMP_WORKER_TIME_S 10

// Manter o programa ativo - keep alive in seconds
#define MQTT_KEEP_ALIVE_S 60

// QoS - mqtt_subscribe
// At most once (QoS 0)
// At least once (QoS 1)
// Exactly once (QoS 2)
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 1
#define MQTT_PUBLISH_RETAIN 0

// Tópico usado para: last will and testament
#define MQTT_WILL_TOPIC "/online"
#define MQTT_WILL_MSG "0"
#define MQTT_WILL_QOS 1

#ifndef MQTT_DEVICE_NAME
#define MQTT_DEVICE_NAME "pico"
#endif

// Definir como 1 para adicionar o nome do cliente aos tópicos, para suportar vários dispositivos que utilizam o mesmo servidor
#ifndef MQTT_UNIQUE_TOPIC
#define MQTT_UNIQUE_TOPIC 0
#endif

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */

//Leitura de temperatura do microcotrolador
static float read_onboard_temperature(const char unit);

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err);

// Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name);

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on);

// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state);

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err);

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err);

// Tópicos de assinatura
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub);

// Dados de entrada MQTT
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);

// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker);
static async_at_time_worker_t temperature_worker = { .do_work = temperature_worker_fn };

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state);

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg);

int main(void) {

    // Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();

    //atribui os valores iniciais à pio estática
    my_pio.pin = 7;
    my_pio.address = 0;
    my_pio.offset = 0;
    my_pio.state_machine = 0;

    //configura a pio estática
    config_pio(&my_pio);
    //Ativa pwm nos leds e no buzzer - ver funções da linha 784
    led_pwm();
    buzzer_pwm();
    // pwm_set_gpio_level(BUZZ_A, PWM_WRAP * 0.75);
    // sleep_ms(200);
    // pwm_set_gpio_level(BUZZ_A, 0);

    INFO_printf("mqtt client starting\n");
    pwm_set_gpio_level(LED_B, PWM_WRAP * 0.5);

    // Inicializa o conversor ADC (a leitura do sensor de temperatura interna foi trocada para as leituras do joystick - ver função da linha 516)
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    // Cria registro com os dados do cliente
    static MQTT_CLIENT_DATA_T state;

    // Inicializa a arquitetura do cyw43
    if (cyw43_arch_init()) {
        pwm_set_gpio_level(LED_B, 0);
        pwm_set_gpio_level(LED_R, PWM_WRAP * 0.5);
        panic("Failed to inizialize CYW43");
    }

    // Usa identificador único da placa
    char unique_id_buf[5];
    pico_get_unique_board_id_string(unique_id_buf, sizeof(unique_id_buf));
    for(int i=0; i < sizeof(unique_id_buf) - 1; i++) {
        unique_id_buf[i] = tolower(unique_id_buf[i]);
    }

    // Gera nome único, Ex: pico1234
    char client_id_buf[sizeof(MQTT_DEVICE_NAME) + sizeof(unique_id_buf) - 1];
    memcpy(&client_id_buf[0], MQTT_DEVICE_NAME, sizeof(MQTT_DEVICE_NAME) - 1);
    memcpy(&client_id_buf[sizeof(MQTT_DEVICE_NAME) - 1], unique_id_buf, sizeof(unique_id_buf) - 1);
    client_id_buf[sizeof(client_id_buf) - 1] = 0;
    INFO_printf("Device name %s\n", client_id_buf);

    state.mqtt_client_info.client_id = client_id_buf;
    state.mqtt_client_info.keep_alive = MQTT_KEEP_ALIVE_S; // Keep alive in sec
#if defined(MQTT_USERNAME) && defined(MQTT_PASSWORD)
    state.mqtt_client_info.client_user = MQTT_USERNAME;
    state.mqtt_client_info.client_pass = MQTT_PASSWORD;
#else
    state.mqtt_client_info.client_user = NULL;
    state.mqtt_client_info.client_pass = NULL;
#endif
    static char will_topic[MQTT_TOPIC_LEN];
    strncpy(will_topic, full_topic(&state, MQTT_WILL_TOPIC), sizeof(will_topic));
    state.mqtt_client_info.will_topic = will_topic;
    state.mqtt_client_info.will_msg = MQTT_WILL_MSG;
    state.mqtt_client_info.will_qos = MQTT_WILL_QOS;
    state.mqtt_client_info.will_retain = true;
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // TLS enabled
#ifdef MQTT_CERT_INC
    static const uint8_t ca_cert[] = TLS_ROOT_CERT;
    static const uint8_t client_key[] = TLS_CLIENT_KEY;
    static const uint8_t client_cert[] = TLS_CLIENT_CERT;
    // This confirms the indentity of the server and the client
    state.mqtt_client_info.tls_config = altcp_tls_create_config_client_2wayauth(ca_cert, sizeof(ca_cert),
            client_key, sizeof(client_key), NULL, 0, client_cert, sizeof(client_cert));
#if ALTCP_MBEDTLS_AUTHMODE != MBEDTLS_SSL_VERIFY_REQUIRED
    WARN_printf("Warning: tls without verification is insecure\n");
#endif
#else
    state->client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
    WARN_printf("Warning: tls without a certificate is insecure\n");
#endif
#endif

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    cyw43_arch_enable_sta_mode();
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        pwm_set_gpio_level(LED_B, 0);
        pwm_set_gpio_level(LED_R, PWM_WRAP * 0.5);
        panic("Failed to connect");
    }
    INFO_printf("\nConnected to Wifi\n");
    pwm_set_gpio_level(LED_B, 0);
    pwm_set_gpio_level(LED_G, PWM_WRAP * 0.5);

    //Faz um pedido de DNS para o endereço IP do servidor MQTT
    cyw43_arch_lwip_begin();
    int err = dns_gethostbyname(MQTT_SERVER, &state.mqtt_server_address, dns_found, &state);
    cyw43_arch_lwip_end();

    // Se tiver o endereço, inicia o cliente (linha 707)
    if (err == ERR_OK) {
        start_client(&state);
    } else if (err != ERR_INPROGRESS) { // ERR_INPROGRESS means expect a callback
        pwm_set_gpio_level(LED_B, 0);
        pwm_set_gpio_level(LED_G, 0);
        pwm_set_gpio_level(LED_R, PWM_WRAP * 0.5);
        panic("dns request failed");
    }

    // Loop condicionado a conexão mqtt
    while (!state.connect_done || mqtt_client_is_connected(state.mqtt_client_inst)) {
        pwm_set_gpio_level(LED_G, PWM_WRAP * 0.5);
        cyw43_arch_poll();
        cyw43_arch_wait_for_work_until(make_timeout_time_ms(10000));
    }
    pwm_set_gpio_level(LED_B, 0);
    pwm_set_gpio_level(LED_G, 0);
    pwm_set_gpio_level(LED_R, 0);
    INFO_printf("mqtt client exiting\n");
    return 0;
}

/* References for this implementation:
 * raspberry-pi-pico-c-sdk.pdf, Section '4.1.1. hardware_adc'
 * pico-examples/adc/adc_console/adc_console.c */
static float read_onboard_temperature(const char unit) {

    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C' || unit != 'F') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

static float read_humidity(const char unit) {

    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    if (unit == 'C' || unit != 'F') {
        return tempC;
    } else if (unit == 'F') {
        return tempC * 9 / 5 + 32;
    }

    return -1.0f;
}

// Requisição para publicar
static void pub_request_cb(__unused void *arg, err_t err) {
    if (err != 0) {
        ERROR_printf("pub_request_cb failed %d", err);
    }
}

//Topico MQTT
static const char *full_topic(MQTT_CLIENT_DATA_T *state, const char *name) {
#if MQTT_UNIQUE_TOPIC
    static char full_topic[MQTT_TOPIC_LEN];
    snprintf(full_topic, sizeof(full_topic), "/%s%s", state->mqtt_client_info.client_id, name);
    return full_topic;
#else
    return name;
#endif
}

// Controle do LED 
static void control_led(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on){
        sketch sketch = {
            .main_color = {
                .blue = 0.1 , .green = 0.1, .red = 0.1
            },
            .figure = {
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1
            } 
        };
        draw(sketch, 0, my_pio, 25);
    } else {
        sketch sketch = {
            .main_color = {
                .blue = 0.1 , .green = 0.1, .red = 0.1
            },
            .figure = {
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0
            } 
        };
        draw(sketch, 0, my_pio, 25);
    }

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/led/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

static void control_buzzer(MQTT_CLIENT_DATA_T *state, bool on) {
    // Publish state on /state topic and on/off led board
    const char* message = on ? "On" : "Off";
    if (on){
        pwm_set_gpio_level(BUZZ_A, 0.5 * PWM_WRAP);
        pwm_set_gpio_level(BUZZ_B, 0.5 * PWM_WRAP);
        sleep_ms(500);
        pwm_set_gpio_level(BUZZ_A, 0);
        pwm_set_gpio_level(BUZZ_B, 0);
    } else {
        pwm_set_gpio_level(BUZZ_A, 0);
        pwm_set_gpio_level(BUZZ_B, 0);
    }

    mqtt_publish(state->mqtt_client_inst, full_topic(state, "/ring/state"), message, strlen(message), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
}

static void control_water(MQTT_CLIENT_DATA_T *state, bool on) {
    const char* message = on ? "On" : "Off";

    if (on){
        sketch sketch0 = {
            .main_color = {
                .blue = 0.05 , .green = 0.01 , .red = 0.01 
            },
            .figure = {
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0, 0
            } 
        };
        sketch sketch1 = {
            .main_color = {
                .blue = 0.05 , .green = 0.01 , .red = 0.01 
            },
            .figure = {
                0, 0, 0, 0, 0,
                0, 1, 1, 1, 0,
                0, 1, 1, 1, 0,
                0, 0, 1, 0, 0,
                0, 0, 0, 0, 0
            } 
        };

        sketch sketch2 = {
            .main_color = {
                .blue = 0.05 , .green = 0.01 , .red = 0.01 
            },
            .figure = {
                0, 1, 1, 1, 0,
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1,
                0, 1, 1, 1, 0,
                0, 0, 1, 0, 0
            } 
        };

        for (uint8_t i = 0; i < 6; i++){
            draw(sketch1, 0, my_pio, 25);
            pwm_set_gpio_level(BUZZ_A, PWM_WRAP * 0.1);
            sleep_ms(100);
            pwm_set_gpio_level(BUZZ_A, 0);
            sleep_ms(100);
            draw(sketch2, 0, my_pio, 25);
            pwm_set_gpio_level(BUZZ_B, PWM_WRAP * 0.1);
            sleep_ms(100);
            pwm_set_gpio_level(BUZZ_B, 0);
            sleep_ms(100);
            draw(sketch0, 0, my_pio, 25);
            sleep_ms(100);
        }
    } else {
        sketch sketch = {
            .main_color = {
                .blue = 0.0, .green = 0.0, .red = 0.0
            },
            .figure = {
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1,
                1, 1, 1, 1, 1
            } 
        };
        draw(sketch, 0, my_pio, 25);
    }
}


// Publicar temperatura
static void publish_temperature(MQTT_CLIENT_DATA_T *state) {
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    float temperature = read_onboard_temperature(TEMPERATURE_UNITS);
    if (temperature != old_temperature) {
        old_temperature = temperature;
        // Publish temperature on /temperature topic
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        INFO_printf("Publishing %s to %s\n", temp_str, temperature_key);
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
}

//Tópico que faz as leituras de temperatura e umidade com o joystick
static void publish_humidity(MQTT_CLIENT_DATA_T *state) {
    static float old_humidity;
    static float old_temperature;
    const char *temperature_key = full_topic(state, "/temperature");
    const char *humidity_key = full_topic(state, "/humidity");
    adc_select_input(1); // GPIO 26 = ADC0
    float temperature = (adc_read() * 50) / 4095;
    adc_select_input(0); // GPIO 26 = ADC0
    int humidity = (adc_read() * 100) / 4095;
    if (temperature != old_temperature) {
        old_temperature = temperature;
        // Publish temperature on /temperature topic
        char temp_str[16];
        snprintf(temp_str, sizeof(temp_str), "%.2f", temperature);
        INFO_printf("Publishing %s to %s\n", temp_str, temperature_key);
        mqtt_publish(state->mqtt_client_inst, temperature_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
    if (humidity != old_humidity){
        old_humidity = humidity;
        // Publish humidity on /humidity topic
        char hum_str[16];
        snprintf(hum_str, sizeof(hum_str), "%d", humidity);
        INFO_printf("Publishing %s to %s\n", hum_str, humidity_key);
        mqtt_publish(state->mqtt_client_inst, humidity_key, hum_str, strlen(hum_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    }
    if (humidity < 30){
        char hum_str[16] = "SECO!\n"; 
        const char *print_key = full_topic(state, "/humNotify");
        INFO_printf(hum_str);
        mqtt_publish(state->mqtt_client_inst, print_key, hum_str, strlen(hum_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (humidity > 60){
        char hum_str[16] = "UMIDO!\n"; 
        const char *print_key = full_topic(state, "/humNotify");
        INFO_printf(hum_str);
        mqtt_publish(state->mqtt_client_inst, print_key, hum_str, strlen(hum_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);       
    } else {
        char hum_str[16] = "bom\n"; 
        const char *print_key = full_topic(state, "/humNotify");
        INFO_printf(hum_str);
        mqtt_publish(state->mqtt_client_inst, print_key, hum_str, strlen(hum_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);       
    }
    if (temperature > 35){
        char temp_str[16] = "QUENTE!\n"; 
        const char *print_key = full_topic(state, "/tempNotify");
        INFO_printf(temp_str);
        mqtt_publish(state->mqtt_client_inst, print_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (temperature < 15){
        char temp_str[16] = "FRIO!\n"; 
        const char *print_key = full_topic(state, "/tempNotify");
        INFO_printf(temp_str);
        mqtt_publish(state->mqtt_client_inst, print_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);  
    } else {
        char temp_str[16] = "bom\n"; 
        const char *print_key = full_topic(state, "/tempNotify");
        INFO_printf(temp_str);
        mqtt_publish(state->mqtt_client_inst, print_key, temp_str, strlen(temp_str), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);      
    }
}

// Requisição de Assinatura - subscribe
static void sub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("subscribe request failed %d", err);
    }
    state->subscribe_count++;
}

// Requisição para encerrar a assinatura
static void unsub_request_cb(void *arg, err_t err) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (err != 0) {
        panic("unsubscribe request failed %d", err);
    }
    state->subscribe_count--;
    assert(state->subscribe_count >= 0);

    // Stop if requested
    if (state->subscribe_count <= 0 && state->stop_client) {
        mqtt_disconnect(state->mqtt_client_inst);
    }
}

// Tópicos de assinatura (ver linha 678)
static void sub_unsub_topics(MQTT_CLIENT_DATA_T* state, bool sub) {
    mqtt_request_cb_t cb = sub ? sub_request_cb : unsub_request_cb;
    //Tópico de acionamento da luminária
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/led"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    //Tópico de acionamento da campainha
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ring"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    //Tópico de acionamento da mangueira de água
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/water"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/print"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    // Tópicos que representam os valores qualitativos das leituras de temperatura e umidade (temp- e hum- Notify)
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/tempNotify"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/humNotify"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/ping"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
    mqtt_sub_unsub(state->mqtt_client_inst, full_topic(state, "/exit"), MQTT_SUBSCRIBE_QOS, cb, state, sub);
}

// Dados de entrada MQTT - Aqui é realizado o controle dos periféricos
static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
#if MQTT_UNIQUE_TOPIC
    const char *basic_topic = state->topic + strlen(state->mqtt_client_info.client_id) + 1;
#else
    const char *basic_topic = state->topic;
#endif
    strncpy(state->data, (const char *)data, len);
    state->len = len;
    state->data[len] = '\0';

    DEBUG_printf("Topic: %s, Message: %s\n", state->topic, state->data);
    if (strcmp(basic_topic, "/led") == 0)
    {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_led(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_led(state, false);
    } else if (strcmp(basic_topic, "/ring") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_buzzer(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_buzzer(state, false);  
    }else if (strcmp(basic_topic, "/water") == 0) {
        if (lwip_stricmp((const char *)state->data, "On") == 0 || strcmp((const char *)state->data, "1") == 0)
            control_water(state, true);
        else if (lwip_stricmp((const char *)state->data, "Off") == 0 || strcmp((const char *)state->data, "0") == 0)
            control_water(state, false);  
    } else if (strcmp(basic_topic, "/print") == 0) {
        INFO_printf("%.*s\n", len, data);
    } else if (strcmp(basic_topic, "/ping") == 0) {
        char buf[11];
        snprintf(buf, sizeof(buf), "%u", to_ms_since_boot(get_absolute_time()) / 1000);
        mqtt_publish(state->mqtt_client_inst, full_topic(state, "/uptime"), buf, strlen(buf), MQTT_PUBLISH_QOS, MQTT_PUBLISH_RETAIN, pub_request_cb, state);
    } else if (strcmp(basic_topic, "/exit") == 0) {
        state->stop_client = true; // stop the client when ALL subscriptions are stopped
        sub_unsub_topics(state, false); // unsubscribe
    }
}



// Dados de entrada publicados
static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    strncpy(state->topic, topic, sizeof(state->topic));
}

// Publicar temperatura
static void temperature_worker_fn(async_context_t *context, async_at_time_worker_t *worker) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)worker->user_data;
    //publish_temperature(state);
    publish_humidity(state);
    async_context_add_at_time_worker_in_ms(context, worker, TEMP_WORKER_TIME_S * 1000);
}

// Conexão MQTT
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    MQTT_CLIENT_DATA_T* state = (MQTT_CLIENT_DATA_T*)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        state->connect_done = true;
        //Aqui realiza a assinatura nos tópicos (ver linha 599)
        sub_unsub_topics(state, true); // subscribe;

        // indicate online
        if (state->mqtt_client_info.will_topic) {
            mqtt_publish(state->mqtt_client_inst, state->mqtt_client_info.will_topic, "1", 1, MQTT_WILL_QOS, true, pub_request_cb, state);
        }

        // Publish temperature every 10 sec if it's changed
        temperature_worker.user_data = state;
        async_context_add_at_time_worker_in_ms(cyw43_arch_async_context(), &temperature_worker, 0);
    } else if (status == MQTT_CONNECT_DISCONNECTED) {
        if (!state->connect_done) {
            pwm_set_gpio_level(LED_B, 0);
            pwm_set_gpio_level(LED_G, 0);
            pwm_set_gpio_level(LED_R, PWM_WRAP * 0.5);
            panic("Failed to connect to mqtt server");
        }
    }
    else {
        pwm_set_gpio_level(LED_B, 0);
        pwm_set_gpio_level(LED_G, 0);
        pwm_set_gpio_level(LED_R, PWM_WRAP * 0.5);
        panic("Unexpected status");
    }
}

// Inicializar o cliente MQTT
static void start_client(MQTT_CLIENT_DATA_T *state) {
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    const int port = MQTT_TLS_PORT;
    INFO_printf("Using TLS\n");
#else
    const int port = MQTT_PORT;
    INFO_printf("Warning: Not using TLS\n");
#endif

    state->mqtt_client_inst = mqtt_client_new();
    if (!state->mqtt_client_inst) {
        pwm_set_gpio_level(LED_B, 0);
        pwm_set_gpio_level(LED_G, 0);
        pwm_set_gpio_level(LED_R, PWM_WRAP * 0.5);
        panic("MQTT client instance creation error");
    }
    INFO_printf("IP address of this device %s\n", ipaddr_ntoa(&(netif_list->ip_addr)));
    INFO_printf("Connecting to mqtt server at %s\n", ipaddr_ntoa(&state->mqtt_server_address));

    cyw43_arch_lwip_begin();
    if (mqtt_client_connect(state->mqtt_client_inst, &state->mqtt_server_address, port, mqtt_connection_cb, state, &state->mqtt_client_info) != ERR_OK) {
        pwm_set_gpio_level(LED_B, 0);
        pwm_set_gpio_level(LED_G, 0);
        pwm_set_gpio_level(LED_R, PWM_WRAP * 0.5);
        panic("MQTT broker connection error");
    }
#if LWIP_ALTCP && LWIP_ALTCP_TLS
    // This is important for MBEDTLS_SSL_SERVER_NAME_INDICATION
    mbedtls_ssl_set_hostname(altcp_tls_context(state->mqtt_client_inst->conn), MQTT_SERVER);
#endif
//Aqui é colocado o callback da publicação de dados no protocolo MQTT (ver linha 617)
    mqtt_set_inpub_callback(state->mqtt_client_inst, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, state);
    cyw43_arch_lwip_end();
}

// Call back com o resultado do DNS
static void dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    MQTT_CLIENT_DATA_T *state = (MQTT_CLIENT_DATA_T*)arg;
    if (ipaddr) {
        state->mqtt_server_address = *ipaddr;
        start_client(state);
    } else {
        pwm_set_gpio_level(LED_B, 0);
        pwm_set_gpio_level(LED_G, 0);
        pwm_set_gpio_level(LED_R, PWM_WRAP * 0.5);
        panic("dns request failed");
    }
}

void config_pio(pio_ref* pio){
    pio->address = pio0;
    if (!set_sys_clock_khz(128000, false))
        printf("clock errado!");
    pio->offset = pio_add_program(pio->address, &pio_review_program);
    pio->state_machine = pio_claim_unused_sm(pio->address, true);

    pio_review_program_init(pio->address, pio->state_machine, pio->offset, pio->pin);
}

uint32_t rgb_matrix(rgb color){
    unsigned char r, g, b;
    r = color.red* 255;
    g = color.green * 255;
    b = color.blue * 255;
    return (g << 24) | (r << 16) | (b << 8);
}

void draw(sketch sketch, uint32_t led_cfg, pio_ref pio, const uint8_t vector_size){
    for(int16_t i = 0; i < vector_size; i++){
        if (sketch.figure[i] == 1)
            led_cfg = rgb_matrix(sketch.main_color);
        else
            led_cfg = 0;
        pio_sm_put_blocking(pio.address, pio.state_machine, led_cfg);
    }
};

void led_pwm(){
    gpio_set_function(LED_R, GPIO_FUNC_PWM);
    uint8_t slice_r = pwm_gpio_to_slice_num(LED_R);

    pwm_set_wrap(slice_r, PWM_WRAP);
    pwm_set_clkdiv(slice_r, PWM_CLKDIV);
    pwm_set_enabled(slice_r, true);

    gpio_set_function(LED_G, GPIO_FUNC_PWM);
    uint8_t slice_g = pwm_gpio_to_slice_num(LED_G);

    pwm_set_wrap(slice_g, PWM_WRAP);
    pwm_set_clkdiv(slice_g, PWM_CLKDIV);
    pwm_set_enabled(slice_g, true);

    gpio_set_function(LED_B, GPIO_FUNC_PWM);
    uint8_t slice_b = pwm_gpio_to_slice_num(LED_B);

    pwm_set_wrap(slice_b, PWM_WRAP);
    pwm_set_clkdiv(slice_b, PWM_CLKDIV);
    pwm_set_enabled(slice_b, true);

}

void buzzer_pwm(){
    gpio_set_function(BUZZ_A, GPIO_FUNC_PWM);
    uint8_t slice_ba = pwm_gpio_to_slice_num(BUZZ_A);

    pwm_set_wrap(slice_ba, PWM_WRAP);
    pwm_set_clkdiv(slice_ba, PWM_CLKDIV);
    pwm_set_enabled(slice_ba, true);

    gpio_set_function(BUZZ_B, GPIO_FUNC_PWM);
    uint8_t slice_bb = pwm_gpio_to_slice_num(BUZZ_B);

    pwm_set_wrap(slice_bb, PWM_WRAP);
    pwm_set_clkdiv(slice_bb, PWM_CLKDIV);
    pwm_set_enabled(slice_bb, true);

}

void joystick_adc(){
    adc_init();
    adc_gpio_init(JOY_X);
    adc_gpio_init(JOY_Y);
}