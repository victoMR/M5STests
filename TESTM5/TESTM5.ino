/**
 * M5StickC Multi-Sensor Monitor con ESP-NOW
 * 
 * Sistema de monitoreo multi-función con capacidad de comunicación ESP-NOW
 * para compartir datos con otros dispositivos ESP32.
 * 
 * Características principales:
 * - Monitoreo de IMU (acelerómetro y giroscopio)
 * - Lectura de micrófono
 * - Estado de batería
 * - Reloj en tiempo real
 * - Sensor de temperatura
 * - Diagnóstico del sistema
 * - Configuración de brillo
 * - Detección de movimiento
 * - Comunicación ESP-NOW
 */

#include <M5StickC.h>
#include <esp_now.h>
#include <WiFi.h>

// Constantes para la configuración
const int NUM_PANTALLAS = 10;
const int TAMANO_BUFFER_ACEL = 10;
const float UMBRAL_MOVIMIENTO = 1.5;
const int PIN_MICROFONO = 36;
const int BRILLO_INICIAL = 128;
const int BRILLO_MIN = 0;
const int BRILLO_MAX = 255;
const int BRILLO_PASO = 10;
const int INTERVALO_REFRESCO = 50;

// Constantes para cálculos de batería
const float VOLTAJE_MAX_BATERIA = 4.2;
const float VOLTAJE_MIN_BATERIA = 3.0;

// MAC Address del ESP32 receptor (cambiar según  dispositivo)
uint8_t dispositivoReceptor[] = {0x50, 0x02, 0x91, 0xF6, 0x16, 0x40};

// Estructura para datos a enviar por ESP-NOW
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

// Estructura para datos del IMU
struct DatosIMU {
    float accX = 0, accY = 0, accZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0;
    float pitch = 0, roll = 0, yaw = 0;
};

// Variables globales para ESP-NOW
char estadoEnvio[20] = "No enviado";
uint8_t ultimoEstadoEnvio = 0;

// Clase principal para gestionar el dispositivo
class M5StickMonitor {
private:
    DatosIMU imu;
    float datosAceleracion[TAMANO_BUFFER_ACEL] = {0};
    int brillo;
    int pantallaActual;
    int valorMicrofono;
    float temperatura;
    unsigned long ultimaActualizacion;
    bool espNowIniciado;
    unsigned long ultimoEnvioESPNow;
    const int INTERVALO_REFRESCO_PANTALLA = 250; // ms entre actualizaciones de pantalla (más suave)
    const float ALPHA_SUAVIZADO = 0.2;   // Factor de suavizado para las lecturas (0-1)
    const float FACTOR_SUAVIZADO = 0.15;
    const unsigned long INTERVALO_ENVIO_ESPNOW = 5000; // 5 segundos entre envíos
    unsigned long ultimoClickTiempo = 0;
    bool esperandoSegundoClick = false;
    float brilloActual;
    float brilloObjetivo;
    float microfonoSuavizado;
    float temperaturaSuavizada;
    bool pantallaInvertida = false;
    const float UMBRAL_ROTACION = 60.0; // Grados para activar la rotación

    // Agregar la constante TIEMPO_DOBLE_CLICK junto con las otras constantes de clase
    static const unsigned long TIEMPO_DOBLE_CLICK = 300; // ms entre clicks

    void limpiarArea(int x, int y, int w, int h) {
        M5.Lcd.fillRect(x, y, w, h, BLACK);
    }

    int calcularPorcentajeBateria(float voltaje) {
        float porcentaje = ((voltaje - VOLTAJE_MIN_BATERIA) / 
                           (VOLTAJE_MAX_BATERIA - VOLTAJE_MIN_BATERIA)) * 100.0;
        return constrain(static_cast<int>(porcentaje), 0, 100);
    }

    void actualizarDatosIMU() {
        M5.IMU.getAccelData(&imu.accX, &imu.accY, &imu.accZ);
        M5.IMU.getGyroData(&imu.gyroX, &imu.gyroY, &imu.gyroZ);
        M5.IMU.getAhrsData(&imu.pitch, &imu.roll, &imu.yaw);
        
        // Actualizar buffer de aceleración para detección de movimiento
        for(int i = TAMANO_BUFFER_ACEL - 1; i > 0; i--) {
            datosAceleracion[i] = datosAceleracion[i-1];
        }
        datosAceleracion[0] = sqrt(imu.accX*imu.accX + imu.accY*imu.accY + imu.accZ*imu.accZ);
    }

    void actualizarDatosSensores() {
        valorMicrofono = analogRead(PIN_MICROFONO);
        temperatura = M5.Axp.GetTempInAXP192();
        
        // Actualizar valores suavizados
        microfonoSuavizado = ALPHA_SUAVIZADO * valorMicrofono + 
                            (1 - ALPHA_SUAVIZADO) * microfonoSuavizado;
        temperaturaSuavizada = ALPHA_SUAVIZADO * temperatura + 
                              (1 - ALPHA_SUAVIZADO) * temperaturaSuavizada;

        // Envío automático de datos ESP-NOW
        if (millis() - ultimoEnvioESPNow >= INTERVALO_ENVIO_ESPNOW) {
            enviarDatosESPNow();
            ultimoEnvioESPNow = millis();
        }
    }

    void actualizarTransicionesSuaves() {
        // Suavizar brillo
        if (abs(brilloActual - brilloObjetivo) > 0.1) {
            brilloActual += (brilloObjetivo - brilloActual) * FACTOR_SUAVIZADO;
            M5.Axp.ScreenBreath(static_cast<int>(brilloActual));
        }
    }

    void inicializarESPNow() {
        // Inicializar Serial y WiFi
        Serial.begin(115200);
        Serial.println("\n\nIniciando ESP-NOW...");
        
        // Desactivar WiFi completamente
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        delay(100);

        // Inicializar WiFi en modo STA
        WiFi.mode(WIFI_STA);
        WiFi.disconnect();    // Asegurar que no está conectado a ninguna red
        delay(100);

        // Imprimir MAC para debugging
        Serial.print("MAC del M5StickC: ");
        Serial.println(WiFi.macAddress());

        // Limpiar cualquier configuración previa de ESP-NOW
        esp_now_deinit();
        delay(100);

        // Inicializar ESP-NOW
        if (esp_now_init() != ESP_OK) {
            Serial.println("Error en esp_now_init()");
            strcpy(estadoEnvio, "Error init");
            espNowIniciado = false;
            return;
        }

        // Registrar callback
        esp_now_register_send_cb([](const uint8_t *mac_addr, esp_now_send_status_t status) {
            ultimoEstadoEnvio = status;
            if (status == ESP_NOW_SEND_SUCCESS) {
                strcpy(estadoEnvio, "Enviado OK");
                Serial.println("Envío exitoso");
            } else {
                strcpy(estadoEnvio, "Error envio");
                Serial.println("Error en envío");
            }
        });

        // Configurar peer info
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, dispositivoReceptor, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        // Agregar peer
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Error agregando peer");
            strcpy(estadoEnvio, "Error peer");
            espNowIniciado = false;
            return;
        }

        espNowIniciado = true;
        strcpy(estadoEnvio, "Listo");
        Serial.println("ESP-NOW iniciado correctamente");

        // Imprimir configuración final
        Serial.print("Receptor configurado: ");
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 dispositivoReceptor[0], dispositivoReceptor[1], dispositivoReceptor[2],
                 dispositivoReceptor[3], dispositivoReceptor[4], dispositivoReceptor[5]);
        Serial.println(macStr);
    }

    void enviarDatosESPNow() {
        if (!espNowIniciado) {
            Serial.println("ESP-NOW no iniciado, reintentando inicialización...");
            inicializarESPNow();  // Reintentar inicialización
            if (!espNowIniciado) {
                strcpy(estadoEnvio, "No iniciado");
                return;
            }
        }

        DatosESPNow datos;
        datos.temperatura = temperatura;
        datos.voltajeBateria = M5.Axp.GetBatVoltage();
        datos.aceleracionX = imu.accX;
        datos.aceleracionY = imu.accY;
        datos.aceleracionZ = imu.accZ;
        datos.timestamp = millis();
        datos.nivelMicrofono = valorMicrofono;
        datos.temperatura_cpu = M5.Axp.GetTempInAXP192();
        datos.porcentaje_bateria = calcularPorcentajeBateria(datos.voltajeBateria);

        Serial.println("Intentando enviar datos...");
        esp_err_t resultado = esp_now_send(dispositivoReceptor, 
                                         (uint8_t*)&datos, 
                                         sizeof(DatosESPNow));
        
        if (resultado != ESP_OK) {
            Serial.println("Error al enviar datos");
            strcpy(estadoEnvio, "Error envio");
            espNowIniciado = false;  // Marcar como no iniciado para reintentar
        } else {
            Serial.println("Datos enviados, esperando callback...");
            strcpy(estadoEnvio, "Enviando...");
        }
    }

    void configurarPantalla() {
        M5.Lcd.setRotation(3); // Rotación inicial normal
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Axp.ScreenBreath(brilloActual);
        pantallaInvertida = false;
    }

    void actualizarRotacionPantalla() {
        // Usar el pitch (inclinación) para determinar la orientación
        if (imu.pitch < -UMBRAL_ROTACION && !pantallaInvertida) {
            M5.Lcd.setRotation(1); // Rotación 180 grados
            pantallaInvertida = true;
        } 
        else if (imu.pitch > -UMBRAL_ROTACION && pantallaInvertida) {
            M5.Lcd.setRotation(3); // Rotación normal
            pantallaInvertida = false;
        }
    }

public:
    M5StickMonitor() : 
        brillo(BRILLO_INICIAL),
        pantallaActual(0),
        ultimaActualizacion(0),
        espNowIniciado(false),  // Asegurar que está inicializado
        ultimoEnvioESPNow(0),
        ultimoClickTiempo(0),
        esperandoSegundoClick(false),
        brilloActual(BRILLO_INICIAL),
        brilloObjetivo(BRILLO_INICIAL),
        microfonoSuavizado(0),
        temperaturaSuavizada(0) {
            strcpy(estadoEnvio, "No iniciado");  // Inicializar estado
        }

    void inicializar() {
        // No inicializar Serial aquí, se hace en inicializarESPNow()
        M5.begin();
        M5.IMU.Init();
        M5.Axp.begin();
        configurarPantalla();
        inicializarESPNow();
        
        // Configurar RTC con la hora actual
        RTC_TimeTypeDef TimeStruct;
        TimeStruct.Hours = 1;
        TimeStruct.Minutes = 0;
        TimeStruct.Seconds = 0;
        M5.Rtc.SetTime(&TimeStruct);
        
        RTC_DateTypeDef DateStruct;
        DateStruct.Year = 2024;
        DateStruct.Month = 10;
        DateStruct.Date = 30;
        M5.Rtc.SetData(&DateStruct);

        // Configuración inicial de pantalla
        M5.Lcd.setTextSize(2);
        M5.Lcd.setTextColor(WHITE, BLACK);
        M5.Lcd.fillScreen(BLACK);
    }

    void manejarBotones() {
        unsigned long tiempoActual = millis();
        
        if (M5.BtnA.wasPressed()) {
            if (pantallaActual == 6) { // Pantalla de configuración
                if (esperandoSegundoClick && 
                    (tiempoActual - ultimoClickTiempo) < TIEMPO_DOBLE_CLICK) {
                    esperandoSegundoClick = false;
                    cambiarPantalla(1);
                } else {
                    esperandoSegundoClick = true;
                    ultimoClickTiempo = tiempoActual;
                    brilloObjetivo = min(float(BRILLO_MAX), brilloObjetivo + float(BRILLO_PASO));
                }
            } else if (pantallaActual == 8) {
                enviarDatosESPNow();
            } else {
                cambiarPantalla(1);
            }
        } else if (M5.BtnB.wasPressed()) {
            if (pantallaActual == 6) {
                if (esperandoSegundoClick && 
                    (tiempoActual - ultimoClickTiempo) < TIEMPO_DOBLE_CLICK) {
                    esperandoSegundoClick = false;
                    cambiarPantalla(-1);
                } else {
                    esperandoSegundoClick = true;
                    ultimoClickTiempo = tiempoActual;
                    brilloObjetivo = max(float(BRILLO_MIN), brilloObjetivo - float(BRILLO_PASO));
                }
            } else {
                cambiarPantalla(-1);
            }
        }
        
        // Reset estado de doble click
        if (esperandoSegundoClick && 
            (tiempoActual - ultimoClickTiempo) >= TIEMPO_DOBLE_CLICK) {
            esperandoSegundoClick = false;
        }
    }

    void actualizar() {
        unsigned long tiempoActual = millis();
        if (tiempoActual - ultimaActualizacion >= INTERVALO_REFRESCO) {
            M5.update();
            manejarBotones();
            actualizarDatosIMU();
            actualizarDatosSensores();
            actualizarTransicionesSuaves();
            actualizarRotacionPantalla();
            mostrarPantallaActual();
            ultimaActualizacion = tiempoActual;
        }
    }

    void cambiarPantalla(int direccion) {
        pantallaActual = (pantallaActual + direccion + NUM_PANTALLAS) % NUM_PANTALLAS;
        M5.Lcd.fillScreen(BLACK);
    }

    void mostrarPantallaActual() {
        switch (pantallaActual) {
            case 0: mostrarPantallaIMU(); break;
            case 1: mostrarPantallaMicrofono(); break;
            case 2: mostrarPantallaBateria(); break;
            case 3: mostrarReloj(); break;
            case 4: mostrarPantallaTemperatura(); break;
            case 5: mostrarPantallaDiagnostico(); break;
            case 6: mostrarPantallaConfiguracion(); break;
            case 7: mostrarPantallaAlarmaMovimiento(); break;
            case 8: mostrarPantallaESPNow(); break;
            case 9: mostrarPantallaMAC(); break;
        }
    }

    void mostrarPantallaIMU() {
        limpiarArea(0, 0, 160, 80);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Datos IMU");
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("P:%.1f", imu.pitch);
        M5.Lcd.setCursor(80, 20);
        M5.Lcd.printf("R:%.1f", imu.roll);
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.printf("Y:%.1f", imu.yaw);
        
        // Mostrar aceleración
        M5.Lcd.setCursor(0, 60);
        float accelTotal = sqrt(imu.accX*imu.accX + 
                              imu.accY*imu.accY + 
                              imu.accZ*imu.accZ);
        M5.Lcd.printf("A:%.1f", accelTotal);
    }

    void mostrarPantallaMicrofono() {
        limpiarArea(0, 0, 160, 80);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Microfono");
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("Val:%d", microfonoSuavizado);
        
        // Barra de nivel
        limpiarArea(0, 40, 160, 20);
        int barraAncho = map(microfonoSuavizado, 0, 4095, 0, 160);
        uint16_t color = microfonoSuavizado > 3000 ? RED : 
                        (microfonoSuavizado > 2000 ? YELLOW : GREEN);
        M5.Lcd.fillRect(0, 40, barraAncho, 20, color);
    }

    void mostrarPantallaBateria() {
        limpiarArea(0, 0, 160, 80);
        float voltajeBat = M5.Axp.GetBatVoltage();
        float corrienteBat = M5.Axp.GetBatCurrent();
        int porcentaje = calcularPorcentajeBateria(voltajeBat);
        
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Bateria");
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("V:%.2fV", voltajeBat);
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.printf("%d%%", porcentaje);
        
        // Mostrar corriente con color según carga/descarga
        M5.Lcd.setCursor(0, 60);
        if (corrienteBat > 0) {
            M5.Lcd.setTextColor(RED);
            M5.Lcd.printf("-%.1fmA", corrienteBat);
        } else {
            M5.Lcd.setTextColor(GREEN);
            M5.Lcd.printf("+%.1fmA", -corrienteBat);
        }
        M5.Lcd.setTextColor(WHITE);

        // Barra de porcentaje
        int barraAncho = map(porcentaje, 0, 100, 0, 160);
        uint16_t color = porcentaje > 60 ? GREEN : 
                        (porcentaje > 20 ? YELLOW : RED);
        M5.Lcd.fillRect(0, 80, barraAncho, 10, color);
    }

    void mostrarReloj() {
        limpiarArea(0, 0, 160, 80);
        RTC_TimeTypeDef tiempoRTC;
        RTC_DateTypeDef fechaRTC;
        
        M5.Rtc.GetTime(&tiempoRTC);
        M5.Rtc.GetData(&fechaRTC);
        
        // Título
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Reloj");
        
        // Mostrar hora en formato grande
        M5.Lcd.setTextSize(3);
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("%02d:%02d", tiempoRTC.Hours, tiempoRTC.Minutes);
        
        // Mostrar segundos
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(120, 25);
        M5.Lcd.printf("%02d", tiempoRTC.Seconds);
        
        // Mostrar fecha
        M5.Lcd.setTextSize(2);
        M5.Lcd.setCursor(0, 60);
        M5.Lcd.printf("%04d/%02d/%02d", fechaRTC.Year, fechaRTC.Month, fechaRTC.Date);
    }

    void mostrarPantallaTemperatura() {
        limpiarArea(0, 0, 160, 80);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Temperatura");
        
        // Temperatura del AXP192
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("CPU: %.1fC", temperaturaSuavizada);
        
        // Mostrar barra de temperatura
        int tempColor = temperaturaSuavizada > 45 ? RED : 
                       (temperaturaSuavizada > 35 ? YELLOW : GREEN);
        int barraAncho = map(temperaturaSuavizada, 20, 60, 0, 160);
        barraAncho = constrain(barraAncho, 0, 160);
        
        limpiarArea(0, 40, 160, 20);
        M5.Lcd.fillRect(0, 40, barraAncho, 20, tempColor);
        
        // Mostrar escala
        M5.Lcd.setCursor(0, 60);
        M5.Lcd.printf("20C");
        M5.Lcd.setCursor(120, 60);
        M5.Lcd.printf("90C");
    }

    void mostrarPantallaDiagnostico() {
        limpiarArea(0, 0, 160, 80);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Diagnostico");
        
        // Estado del IMU
        M5.Lcd.setCursor(0, 20);
        bool imuOk = M5.IMU.Init() == 0;
        M5.Lcd.setTextColor(imuOk ? GREEN : RED);
        M5.Lcd.printf("IMU:%s", imuOk ? "OK" : "Err");
        
        // Estado de la batería
        M5.Lcd.setCursor(0, 40);
        bool batOk = M5.Axp.GetWarningLevel() == 0;
        M5.Lcd.setTextColor(batOk ? GREEN : RED);
        M5.Lcd.printf("Bat:%s", batOk ? "OK" : "Bajo");
        
        // Estado de ESP-NOW
        M5.Lcd.setCursor(0, 60);
        M5.Lcd.setTextColor(espNowIniciado ? GREEN : RED);
        M5.Lcd.printf("ESP:%s", espNowIniciado ? "OK" : "Err");
        
        M5.Lcd.setTextColor(WHITE);
    }

    void mostrarPantallaConfiguracion() {
        limpiarArea(0, 0, 160, 80);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Config");
        
        // Mostrar nivel de brillo actual
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.printf("Brillo:%d", brillo);
        
        // Barra de brillo
        limpiarArea(0, 40, 160, 20);
        int barraAncho = map(brillo, BRILLO_MIN, BRILLO_MAX, 0, 160);
        M5.Lcd.fillRect(0, 40, barraAncho, 20, YELLOW);
        
        // Instrucciones
        M5.Lcd.setCursor(0, 60);
        M5.Lcd.setTextSize(1);
        M5.Lcd.println("A:Aumentar B:Reducir");
        M5.Lcd.setTextSize(2);
    }

    void mostrarPantallaAlarmaMovimiento() {
        limpiarArea(0, 0, 160, 80);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("Alarma Mov.");
        
        // Calcular movimiento promedio
        float movimientoPromedio = 0;
        for(int i = 0; i < TAMANO_BUFFER_ACEL; i++) {
            movimientoPromedio += datosAceleracion[i];
        }
        movimientoPromedio /= TAMANO_BUFFER_ACEL;
        
        bool movimientoDetectado = movimientoPromedio > UMBRAL_MOVIMIENTO;
        
        // Mostrar estado
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.setTextColor(movimientoDetectado ? RED : GREEN);
        M5.Lcd.println(movimientoDetectado ? "MOVIMIENTO!" : "Normal");
        M5.Lcd.setTextColor(WHITE);
        
        // Mostrar nivel de movimiento
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.printf("Nivel:%.2f", movimientoPromedio);
        
        // Barra de nivel
        limpiarArea(0, 60, 160, 20);
        int barraAncho = map(movimientoPromedio * 100, 0, UMBRAL_MOVIMIENTO * 100, 0, 160);
        barraAncho = constrain(barraAncho, 0, 160);
        uint16_t color = movimientoDetectado ? RED : GREEN;
        M5.Lcd.fillRect(0, 60, barraAncho, 20, color);
    }

    void mostrarPantallaESPNow() {
        limpiarArea(0, 0, 160, 80);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("ESP-NOW");
        
        // Estado de la conexión
        M5.Lcd.setCursor(0, 20);
        if (espNowIniciado) {
            M5.Lcd.setTextColor(GREEN);
            M5.Lcd.println("Estado: OK");
        } else {
            M5.Lcd.setTextColor(RED);
            M5.Lcd.println("Estado: ERROR");
        }
        M5.Lcd.setTextColor(WHITE);
        
        // Último estado de envío
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.printf("Estado:%s", estadoEnvio);
        
        // Mostrar MAC del receptor
        M5.Lcd.setCursor(0, 60);
        M5.Lcd.setTextSize(1);
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                 dispositivoReceptor[0], dispositivoReceptor[1], dispositivoReceptor[2],
                 dispositivoReceptor[3], dispositivoReceptor[4], dispositivoReceptor[5]);
        M5.Lcd.printf("Rx:%s", macStr);
        
        // Instrucciones
        M5.Lcd.setCursor(0, 70);
        M5.Lcd.println("A:Enviar  Auto:5s");
        M5.Lcd.setTextSize(2);
    }

    void mostrarPantallaMAC() {
        limpiarArea(0, 0, 160, 80);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.println("MAC Address");
        
        // Obtener y mostrar la MAC
        M5.Lcd.setCursor(0, 20);
        M5.Lcd.setTextSize(1); // Texto más pequeño para que quepa la MAC
        String mac = WiFi.macAddress();
        M5.Lcd.println(mac);
        
        // Mostrar instrucciones
        M5.Lcd.setCursor(0, 40);
        M5.Lcd.println("Esta es la MAC");
        M5.Lcd.println("del M5StickC");
        
        M5.Lcd.setTextSize(2); // Restaurar tamaño de texto
    }
};

// Función principal
void setup() {
    static M5StickMonitor monitor;
    monitor.inicializar();
}

void loop() {
    static M5StickMonitor monitor;
    monitor.actualizar();
}