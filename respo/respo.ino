/**
 * ESP8266 Receiver with local logs  // ESP8266 Receptor con logs locales
 */

#include <ESP8266WiFi.h>
#include <espnow.h>

// LED and blink configuration  // Configuración LED y parpadeo
#define LED_BUILTIN_PIN LED_BUILTIN
const int BLINK_OK = 100; // PARPADEO_OK
const int BLINK_ERROR = 500; // PARPADEO_ERROR

// Global variables  // Variables globales
unsigned long lastBlink = 0; // ultimoParpadeo
bool ledState = false; // ledEstado
bool espNowInitialized = false; // espNowIniciado
unsigned long lastReception = 0; // ultimaRecepcion
const unsigned long RECEPTION_TIMEOUT = 10000; // TIMEOUT_RECEPCION

// ESP-NOW data structure  // Estructura de datos ESP-NOW
struct ESPNowData { // DatosESPNow
    float temperature; // temperatura
    float batteryVoltage; // voltajeBateria
    float accelerationX; // aceleracionX
    float accelerationY; // aceleracionY
    float accelerationZ; // aceleracionZ
    uint32_t timestamp; // timestamp
    float microphoneLevel; // nivelMicrofono
    float cpuTemperature; // temperatura_cpu
    uint8_t batteryPercentage; // porcentaje_bateria
};

ESPNowData receivedData; // datosRecibidos

void blinkLed(int interval) { // parpadearLed(int intervalo)
    if (millis() - lastBlink >= interval) { // ultimoParpadeo
        ledState = !ledState; // ledEstado
        digitalWrite(LED_BUILTIN_PIN, ledState); // ledEstado
        lastBlink = millis(); // ultimoParpadeo
    }
}

void initializeESPNow() { // inicializarESPNow()
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    if (esp_now_init() != 0) {
        Serial.println("Error inicializando ESP-NOW");
        espNowInitialized = false; // espNowIniciado
        return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb([](uint8_t *mac, uint8_t *data, uint8_t len) {
        onDataRecv(mac, data, len); // OnDataRecv
    });

    espNowInitialized = true; // espNowIniciado
    Serial.println("ESP-NOW iniciado correctamente");
}

void onDataRecv(uint8_t *macAddr, uint8_t *incomingData, uint8_t len) { // OnDataRecv(uint8_t *mac_addr
    lastReception = millis(); // ultimaRecepcion
    
    if (len != sizeof(ESPNowData)) { // DatosESPNow
        Serial.println("Error: Tamaño datos incorrecto");
        return;
    }

    memcpy(&receivedData, incomingData, sizeof(receivedData)); // datosRecibidos
    
    // Imprimir MAC del emisor  // Print sender MAC address
    Serial.print(
Emisor MAC: ");
    for(int i = 0; i < 6; i++) {
        Serial.printf("%02X", macAddr[i]); // mac_addr
        if(i < 5) Serial.print(":");
    }
    
    // Imprimir datos recibidos  // Print received data
    Serial.println(
----- Datos Recibidos -----");
    Serial.printf("Timestamp: %lu
", receivedData.timestamp); // timestamp
    Serial.printf("Temperatura: %.2f°C
", receivedData.temperature); // temperatura
    Serial.printf("Batería: %.2fV (%d%%)
", 
                 receivedData.batteryVoltage,  // voltajeBateria
                 receivedData.batteryPercentage); // porcentaje_bateria
    Serial.printf("CPU Temp: %.2f°C
", receivedData.cpuTemperature); // temperatura_cpu
    Serial.printf("Micrófono: %.0f
", receivedData.microphoneLevel); // nivelMicrofono
    Serial.printf("Aceleración: X=%.2f Y=%.2f Z=%.2f
",
                 receivedData.accelerationX, // aceleracionX
                 receivedData.accelerationY, // aceleracionY
                 receivedData.accelerationZ); // aceleracionZ
    Serial.println("-------------------------");
}

void setup() {
    pinMode(LED_BUILTIN_PIN, OUTPUT);
    Serial.begin(9600);
    delay(1000);
    Serial.println(
Iniciando receptor ESP-NOW...");  // Starting ESP-NOW receiver...
    
    initializeESPNow(); // inicializarESPNow
    
    Serial.print("MAC Receptor: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
    if (!espNowInitialized) { // espNowIniciado
        Serial.println("Reintentando ESP-NOW...");
        initializeESPNow(); // inicializarESPNow
        blinkLed(BLINK_ERROR); // parpadearLed(PARPADEO_ERROR)
        delay(5000);
        return;
    }

    // Blink LED according to status  // Parpadear LED según estado
    if (millis() - lastReception > RECEPTION_TIMEOUT) { // ultimaRecepcion, TIMEOUT_RECEPCION
        blinkLed(BLINK_ERROR);  // parpadearLed(PARPADEO_ERROR) // Slow blink if no data  // Parpadeo lento si no hay datos
    } else {
        blinkLed(BLINK_OK);     // parpadearLed(PARPADEO_OK) // Fast blink if all OK  // Parpadeo rápido si todo OK
    }

    delay(10);
}
