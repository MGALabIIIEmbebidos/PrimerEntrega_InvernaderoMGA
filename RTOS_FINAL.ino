#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <debounce.h>
#include <RtcDS1302.h>
#include "esp_sleep.h"  // Necesario para deep sleep

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// ====================== CONFIGURACIÓN HARDWARE ======================
// DHT
#define DHTPIN      5
#define DHTTYPE     DHT22

// LDR
#define LDRPIN      32

// LED
#define LED_BLUE    26

// BOTONES
#define BUTTON_DEC  34
#define BUTTON_INC  33

// Buzzer
#define BUZZER_PIN  14

// UMBRALES
#define TEMP_THRESHOLD   26
#define HUM_THRESHOLD    80
#define LIGHT_THRESHOLD  700

// Configuración para Deep Sleep
#define uS_TO_S_FACTOR 1000000  // Conversión: microsegundos a segundos
#define TIME_TO_SLEEP  10       // Tiempo en segundos antes de entrar en deep sleep

// ====================== DEFINICIÓN DE ERRORES ======================
#define ERROR_NONE      0x00
#define ERROR_TEMP      0x01
#define ERROR_HUM       0x02
#define ERROR_LIGHT     0x04

// ====================== RTC DS1302 ======================
ThreeWire myWire(4, 15, 2); // DATOS, CLK, RST
RtcDS1302<ThreeWire> Rtc(myWire);

// ====================== ESTRUCTURA DE DATOS ======================
struct SensorData {
  float temperature;
  float humidity;
  int   light;
  uint8_t errorFlags; // 0: sin error, bits definidos según sensor
};

QueueHandle_t sensorQueue;

// --------------------- ESTRUCTURA PARA EVENTOS DE BOTONES ---------------------
struct ButtonEvent {
  int increment;  // +1 o -1
  char name[4];   // "INC" o "DEC"
};

QueueHandle_t buttonQueue;

// --------------------- VARIABLES GLOBALES ---------------------
volatile float g_temperature = 0.0f;
volatile float g_humidity    = 0.0f;
volatile int   g_light       = 0;
volatile int   counter       = 0; // Contador global modificado por botones

DHT dht(DHTPIN, DHTTYPE);

// --------------------- VARIABLES PARA DEBOUNCE ---------------------
volatile unsigned long lastIncTime = 0;
volatile unsigned long lastDecTime = 0;
const unsigned long debounceDelay = 500000; // 500 ms en microsegundos

// --------------------- MACRO PARA EL TAMAÑO DE UN ARREGLO ---------------------
#define countof(a) (sizeof(a) / sizeof(a[0]))

// --------------------- FUNCIÓN PARA IMPRIMIR CON TIMESTAMP ---------------------
void logWithTimestamp(const char* label, float value, const char* unit)
{
  RtcDateTime now = Rtc.GetDateTime();
  char datestring[26];
  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             now.Month(),
             now.Day(),
             now.Year(),
             now.Hour(),
             now.Minute(),
             now.Second());
  Serial.print("[");
  Serial.print(datestring);
  Serial.print("] ");
  Serial.print(label);
  Serial.print(": ");
  Serial.print(value);
  Serial.print(" ");
  Serial.println(unit);
}

// --------------------- FUNCIÓN PARA IMPRIMIR ERRORES CON TIMESTAMP ---------------------
void logError(const char* errorMsg)
{
  RtcDateTime now = Rtc.GetDateTime();
  char datestring[26];
  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             now.Month(),
             now.Day(),
             now.Year(),
             now.Hour(),
             now.Minute(),
             now.Second());
  Serial.print("[");
  Serial.print(datestring);
  Serial.print("] [ERROR] ");
  Serial.println(errorMsg);
}

// --------------------- FUNCIONES DE WAKEUP ---------------------
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal (ext0)"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

// --------------------- ISR PARA BOTÓN DE INCREMENTO ---------------------
void IRAM_ATTR incISR()
{
  unsigned long nowMicros = micros();
  if(nowMicros - lastIncTime < debounceDelay) return; // Evitar rebotes
  lastIncTime = nowMicros;

  ButtonEvent evt = { +1, "INC" };
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(buttonQueue, &evt, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// --------------------- ISR PARA BOTÓN DE DECREMENTO ---------------------
void IRAM_ATTR decISR()
{
  unsigned long nowMicros = micros();
  if(nowMicros - lastDecTime < debounceDelay) return; // Evitar rebotes
  lastDecTime = nowMicros;

  ButtonEvent evt = { -1, "DEC" };
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(buttonQueue, &evt, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

// ====================== TAREAS ======================

// (1) Tarea de lectura de temperatura con timestamp
void readTemperatureTask(void *pvParameters)
{
  while (1)
  {
    float temp = dht.readTemperature();
    if (!isnan(temp))
    {
      g_temperature = temp;
      logWithTimestamp("Temperature", temp, "°C");
    }
    else {
      logError("Fallo en lectura de temperatura.");
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// (2) Tarea de lectura de humedad con timestamp
void readHumidityTask(void *pvParameters)
{
  while (1)
  {
    float hum = dht.readHumidity();
    if (!isnan(hum))
    {
      g_humidity = hum;
      logWithTimestamp("Humidity", hum, "%");
    }
    else {
      logError("Fallo en lectura de humedad.");
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// (3) Tarea de lectura de luz con timestamp
void readLightTask(void *pvParameters)
{
  while (1)
  {
    int lightVal = analogRead(LDRPIN);
    g_light = lightVal;
    logWithTimestamp("Light", (float)lightVal, "");
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// (4) Tarea que agrega lecturas en la cola y maneja errores en la trama
void aggregatorTask(void *pvParameters)
{
  while (1)
  {
    SensorData data;
    data.temperature = g_temperature;
    data.humidity    = g_humidity;
    data.light       = g_light;
    data.errorFlags  = ERROR_NONE;
    
    // Verifica errores en cada sensor
    if (isnan(data.temperature))
      data.errorFlags |= ERROR_TEMP;
    if (isnan(data.humidity))
      data.errorFlags |= ERROR_HUM;
    // Suponiendo que la lectura del LDR debe estar en el rango 0-4095 (para ESP32)
    if (data.light < 0 || data.light > 4095)
      data.errorFlags |= ERROR_LIGHT;
      
    // Se informa el error con la hora del RTC si se detecta alguna falla
    if (data.errorFlags != ERROR_NONE)
    {
      char errorMsg[64];
      snprintf(errorMsg, sizeof(errorMsg), "Fallo en lectura de sensores. Flags: %02X", data.errorFlags);
      logError(errorMsg);
    }
    
    xQueueSend(sensorQueue, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

// (5) Tarea que controla el LED (alarma) y el buzzer
void controlLedTask(void *pvParameters)
{
  SensorData data;
  while (1)
  {
    if (xQueueReceive(sensorQueue, &data, portMAX_DELAY))
    {
      // Si se detecta algún error en la trama, se evita la activación de la alarma
      if (data.errorFlags != ERROR_NONE)
      {
        logError("Trama con error. No se activa alarma.");
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(BUZZER_PIN, LOW);
      }
      // Evaluación de umbrales en condiciones normales
      else if (data.temperature > TEMP_THRESHOLD || 
               data.humidity    > HUM_THRESHOLD  || 
               data.light       > LIGHT_THRESHOLD)
      {
        digitalWrite(LED_BLUE, HIGH);  // Enciende el LED
        digitalWrite(BUZZER_PIN, HIGH); // Activa el buzzer
        Serial.println("[ALERTA] Umbral superado: LED y Buzzer ACTIVADOS");
      }
      else
      {
        digitalWrite(LED_BLUE, LOW);   // Apaga el LED
        digitalWrite(BUZZER_PIN, LOW);   // Apaga el buzzer
      }
    }
  }
}

// (6) Tarea para manejar eventos de botones (contador)
void buttonEventTask(void *pvParameters)
{
  ButtonEvent evt;
  while (1)
  {
    if (xQueueReceive(buttonQueue, &evt, portMAX_DELAY))
    {
      counter += evt.increment;
      char msg[64];
      snprintf(msg, sizeof(msg), "Button %s pressed, counter: %d", evt.name, counter);
      logWithTimestamp(msg, 0, "");
    }
  }
}

// (7) Tarea para iniciar Deep Sleep con timer y botón como despertador
void deepSleepTask(void *pvParameters)
{
  // Espera el tiempo definido antes de entrar en deep sleep
  vTaskDelay(pdMS_TO_TICKS(TIME_TO_SLEEP * 1000));
  Serial.println("Preparando para Deep Sleep...");
  Serial.flush();

  // Configura el despertador por timer (TIME_TO_SLEEP segundos)
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  // Configura ext0 para que, al detectar que BUTTON_INC pasa a LOW, despierte el ESP32.
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON_INC, 0);

  // Entra en Deep Sleep
  esp_deep_sleep_start();
}

// ====================== SETUP ======================
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  dht.begin();

  // Inicialización del RTC DS1302
  Rtc.Begin();
  Serial.print("compiled: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);

  // Imprime la razón del wakeup
  print_wakeup_reason();

  // Configuración de pines
  pinMode(LED_BLUE, OUTPUT);
  // Configuramos BUTTON_INC para que funcione con ext0: en reposo se mantiene HIGH (INPUT_PULLUP)
  pinMode(BUTTON_INC, INPUT_PULLUP);
  // BUTTON_DEC solo actúa como contador
  pinMode(BUTTON_DEC, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  // Creación de la cola para datos de sensores
  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  if (sensorQueue == NULL)
  {
    logError("Error al crear la cola de sensores.");
    while (1) { delay(100); }
  }

  // Creación de la cola para eventos de botones
  buttonQueue = xQueueCreate(5, sizeof(ButtonEvent));
  if (buttonQueue == NULL)
  {
    logError("Error al crear la cola de botones.");
    while (1) { delay(100); }
  }

  // Configuración de interrupciones para los botones con debounce
  attachInterrupt(digitalPinToInterrupt(BUTTON_INC), incISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_DEC), decISR, FALLING);

  // Crear tareas de FreeRTOS
  xTaskCreatePinnedToCore(readTemperatureTask, "ReadTemp", 2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(readHumidityTask,    "ReadHum",  2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(readLightTask,       "ReadLight",2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(aggregatorTask,      "Aggregator",2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(controlLedTask,      "ControlLED",2048, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(buttonEventTask,     "ButtonEvt",2048, NULL, 2, NULL, app_cpu);
  // Tarea para deep sleep
  xTaskCreatePinnedToCore(deepSleepTask,       "DeepSleep",2048, NULL, 1, NULL, app_cpu);
}

void loop()
{
  // Vacío, ya que las tareas de FreeRTOS gestionan la ejecución
}
