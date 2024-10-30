/**
 * ESP8266 Receptor con logs locales
 */

#include <ESP8266WiFi.h>
#include <espnow.h>

// Configuración LED y parpadeo
#define LED_BUILTIN_PIN LED_BUILTIN
const int PARPADEO_OK = 100;
const int PARPADEO_ERROR = 500;

// Variables globales
unsigned long ultimoParpadeo = 0;
bool ledEstado = false;
bool espNowIniciado = false;
unsigned long ultimaRecepcion = 0;
const unsigned long TIMEOUT_RECEPCION = 10000;

// Estructura de datos ESP-NOW
struct DatosESPNow {
    float temperatura;
    float voltajeBateria;
    float aceleracionX;
    float aceleracionY;
    float aceleracionZ;
    uint32_t timestamp;
    float nivelMicrofono;
    float temperatura_cpu;
    uint8_t porcentaje_bateria;
};

DatosESPNow datosRecibidos;

void parpadearLed(int intervalo) {
    if (millis() - ultimoParpadeo >= intervalo) {
        ledEstado = !ledEstado;
        digitalWrite(LED_BUILTIN_PIN, ledEstado);
        ultimoParpadeo = millis();
    }
}

void inicializarESPNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (esp_now_init() != 0) {
        Serial.println("Error inicializando ESP-NOW");
        espNowIniciado = false;
        return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb([](uint8_t *mac, uint8_t *data, uint8_t len) {
        OnDataRecv(mac, data, len);
    });

    espNowIniciado = true;
    Serial.println("ESP-NOW iniciado correctamente");
}

void OnDataRecv(uint8_t *mac_addr, uint8_t *incomingData, uint8_t len) {
    ultimaRecepcion = millis();
    
    if (len != sizeof(DatosESPNow)) {
        Serial.println("Error: Tamaño datos incorrecto");
        return;
    }

    memcpy(&datosRecibidos, incomingData, sizeof(datosRecibidos));
    
    // Imprimir MAC del emisor
    Serial.print("\nEmisor MAC: ");
    for(int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac_addr[i]);
        if(i < 5) Serial.print(":");
    }
    
    // Imprimir datos recibidos
    Serial.println("\n----- Datos Recibidos -----");
    Serial.printf("Timestamp: %lu\n", datosRecibidos.timestamp);
    Serial.printf("Temperatura: %.2f°C\n", datosRecibidos.temperatura);
    Serial.printf("Batería: %.2fV (%d%%)\n", 
                 datosRecibidos.voltajeBateria, 
                 datosRecibidos.porcentaje_bateria);
    Serial.printf("CPU Temp: %.2f°C\n", datosRecibidos.temperatura_cpu);
    Serial.printf("Micrófono: %.0f\n", datosRecibidos.nivelMicrofono);
    Serial.printf("Aceleración: X=%.2f Y=%.2f Z=%.2f\n",
                 datosRecibidos.aceleracionX,
                 datosRecibidos.aceleracionY,
                 datosRecibidos.aceleracionZ);
    Serial.println("-------------------------");
}

void setup() {
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    Serial.begin(9600);
    delay(1000);
    Serial.println("\nIniciando receptor ESP-NOW...");
    
    inicializarESPNow();
    
    Serial.print("MAC Receptor: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
    if (!espNowIniciado) {
        Serial.println("Reintentando ESP-NOW...");
        inicializarESPNow();
        parpadearLed(PARPADEO_ERROR);
        delay(5000);
        return;
    }

    // Parpadear LED según estado
    if (millis() - ultimaRecepcion > TIMEOUT_RECEPCION) {
        parpadearLed(PARPADEO_ERROR);  // Parpadeo lento si no hay datos
    } else {
        parpadearLed(PARPADEO_OK);     // Parpadeo rápido si todo OK
    }

    delay(10);
} 