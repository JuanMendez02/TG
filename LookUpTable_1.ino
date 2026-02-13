//Se ajusto el formato de struct del evento, y la impresion serial. Se inicia offset en 1650 y se coloca un filtro EMA fuerte, para evitar sobrepicos en cambio de corrientes bruscos.

#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"
#include <WiFi.h>
#include <time.h>
#include "esp_sntp.h"

// ==========================================
// 1. DEFINICIÓN DE ESTRUCTURA
// ==========================================
struct __attribute__((packed)) EventoCompacto {
  uint32_t timestamp; 
  uint16_t rms_x100;  
  uint8_t  codigo;    
  uint8_t  status;    
};

// ==========================================
// 2. CONSTANTES Y CONFIGURACIÓN
// ==========================================
// ADC
const int SENSOR_PIN = 4;
const int SENSOR_PIN_2 = 7;
const int MAX_SAMPLES = 64;
const unsigned long CYCLE_US = 16666UL;
const unsigned long TARGET_DT = CYCLE_US / MAX_SAMPLES;

// Calibración (LUT)
const int PUNTOS_CAL = 7;
const float LUT_IN[]  = { 75.0,  858.0, 1058.0, 1707.0, 2588.0, 2975.0, 3300.0 };
const float LUT_OUT[] = { 100.0, 800.0, 1000.0, 1650.0, 2500.0, 2800.0, 3100.0 };
const float SENSOR_SENS1 = 12.43984f;
const float SENSOR_SENS2 = 12.47566f;
const float K_CORRECCION = 0.28f; 

// Umbrales
const float UMBRALES[] = {12.0, 25.0, 35.0};
const int NUM_UMBRALES = 3;
const float HISTeresis = 1.0; 

// Filtros
const float ALPHA_LENTO = 0.05f;
const float ALPHA_RAPIDO = 0.50f;
const float UMBRAL_REACCION = 15.0f;
const float alphaEMA = 0.005f; // Para el offset

// Códigos y Banderas
#define FLG_SENT 0x01
#define FLG_ACK  0x02
#define BASE_SUBIDA 0x10  
#define BASE_BAJADA 0x20 

// WiFi
const char* ssid     = "univalle";          
const char* password = "Univalle";      
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -18000;        
const int   daylightOffset_sec = 0;        

// ==========================================
// 3. VARIABLES GLOBALES
// ==========================================
// Buffers DMA
#define NUM_BUFFERS 2
volatile uint16_t samples_mV[NUM_BUFFERS][MAX_SAMPLES];
volatile uint16_t samples2_mV[NUM_BUFFERS][MAX_SAMPLES];
volatile uint32_t bufferMicros[NUM_BUFFERS][MAX_SAMPLES];
volatile int activeBuffer = 0;
volatile int sampleIdx1 = 0;
volatile int sampleIdx2 = 0;
volatile bool bufferListo[NUM_BUFFERS] = {false, false};

// FreeRTOS
TaskHandle_t taskProcesarHandle = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Procesamiento
float vPico1_EMA = 0.0f;
float vPico2_EMA = 0.0f;
// Inicializar cerca del valor real esperado para que no empiece en 0
float ema_offset_trim_1 = 1650.0f; 
float ema_offset_trim_2 = 1650.0f;
float I_pico_promedio = 0.0f;
float I_rms_promedio = 0.0f;

// Estadísticas
uint32_t tAdq_us = 0, tProc_us = 0, tTotal_us = 0;
volatile unsigned long ciclosProcesados = 0;
unsigned long ultimoConteo = 0;

// Lógica de Eventos
bool enSC = false;              
int nivelActual = 0; 
EventoCompacto historialBin[100]; 
int idxHist = 0;                  
int countHist = 0;                

// ADC Handle
adc_continuous_handle_t my_adc_handle = NULL;

// ==========================================
// 4. FUNCIONES AUXILIARES
// ==========================================

float obtenerVoltajeReal(float lecturaTeorica) {
    if (lecturaTeorica <= LUT_IN[0]) return LUT_OUT[0]; 
    if (lecturaTeorica >= LUT_IN[PUNTOS_CAL-1]) return LUT_OUT[PUNTOS_CAL-1];

    for (int i = 0; i < PUNTOS_CAL - 1; i++) {
        if (lecturaTeorica >= LUT_IN[i] && lecturaTeorica <= LUT_IN[i+1]) {
            float m = (LUT_OUT[i+1] - LUT_OUT[i]) / (LUT_IN[i+1] - LUT_IN[i]);
            return LUT_OUT[i] + (m * (lecturaTeorica - LUT_IN[i]));
        }
    }
    return 0.0;
}

void setupWiFi() {
  Serial.printf("\nConectando a %s ", ssid);
  WiFi.begin(ssid, password);
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Conectado.");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  } else {
    Serial.println("\nNo se pudo conectar al WiFi.");
  }
}

String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "00/00 00:00:00.000";
  char timeStringBuff[30];
  strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M:%S", &timeinfo);
  unsigned long ms = millis() % 1000;
  char fullTimestamp[40];
  snprintf(fullTimestamp, sizeof(fullTimestamp), "%s.%03lu", timeStringBuff, ms);
  return String(fullTimestamp);
}

void registrarEventoBinario(uint8_t codigoEvento, float valorRMS) {
    if (countHist >= 100) return;

    time_t now;
    time(&now);
    
    historialBin[idxHist].timestamp = (uint32_t)now;
    historialBin[idxHist].rms_x100  = (uint16_t)(valorRMS * 100); 
    historialBin[idxHist].codigo    = codigoEvento;
    historialBin[idxHist].status    = 0x00; 

    Serial.printf("[REGISTRO] Cod: 0x%02X | RMS: %.2f | Time: %u\n", 
                  codigoEvento, valorRMS, (uint32_t)now);

    idxHist++;
    countHist++;
}


void imprimirMemoriaRaw() {
  Serial.println("\n|=== INSPECCIÓN DE MEMORIA (FORMATO BINARIO 8 BYTES) ===|");
  Serial.println("| ID |   HEX (LO QUE SE ENVÍA)     | SEGUNDOS (EPOCH) | FECHA LEGIBLE       | RMS (A) |");
  Serial.println("|----|-----------------------------|------------------|---------------------|---------|");
  
  for (int i = 0; i < countHist; i++) {
    EventoCompacto e = historialBin[i];
    uint8_t* rawBytes = (uint8_t*)&e;

    // 1. Convertir el Timestamp UNIX a fecha legible
    time_t rawTime = (time_t)e.timestamp;
    struct tm * ti;
    ti = localtime(&rawTime); 
    char fechaBuff[20];
    // Formato: DD/MM HH:MM:SS
    sprintf(fechaBuff, "%02d/%02d %02d:%02d:%02d", 
            ti->tm_mday, ti->tm_mon + 1, ti->tm_hour, ti->tm_min, ti->tm_sec);

    Serial.printf("| %02d | ", i);
    
    // 2. Imprimir Hexadecimal (8 Bytes)
    for(int b=0; b<8; b++) {
      Serial.printf("%02X ", rawBytes[b]);
    }

    // 3. Imprimir Segundos Crudos + Fecha Legible + RMS
    // %u imprime el número entero sin signo (tus 1770...)
    Serial.printf("|    %u    | %s |  %5.2f  |\n", 
                  e.timestamp, fechaBuff, (float)e.rms_x100/100.0);
  }
  Serial.println("|===================================================================================|\n");
}


void marcarEnviado(int indice) {
    historialBin[indice].status |= FLG_SENT; 
}

void marcarAck(int indice) {
    historialBin[indice].status |= FLG_ACK;  
}

bool tieneAck(int indice) {
    return (historialBin[indice].status & FLG_ACK);
}

// ==========================================
// 5. INTERRUPCIÓN (ISR) Y PROCESAMIENTO
// ==========================================

static bool IRAM_ATTR adc_dma_isr(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    size_t entries = edata->size / sizeof(adc_digi_output_data_t);
    adc_digi_output_data_t *p = (adc_digi_output_data_t *)edata->conv_frame_buffer;
    portENTER_CRITICAL_ISR(&timerMux);
    for (size_t i = 0; i < entries; i++) {
        uint8_t ch = p[i].type2.channel;
        int val = p[i].type2.data;  
        uint32_t now = micros();
        
        if (ch == ADC_CHANNEL_3 && sampleIdx1 < MAX_SAMPLES) { 
            samples_mV[activeBuffer][sampleIdx1] = val;
            bufferMicros[activeBuffer][sampleIdx1] = now;
            sampleIdx1++;
        }
        else if (ch == ADC_CHANNEL_6 && sampleIdx2 < MAX_SAMPLES) { 
            samples2_mV[activeBuffer][sampleIdx2] = val;
            bufferMicros[activeBuffer][sampleIdx2] = now;
            sampleIdx2++;
        }
        
        if (sampleIdx1 >= MAX_SAMPLES && sampleIdx2 >= MAX_SAMPLES) {
            bufferListo[activeBuffer] = true;
            activeBuffer = 1 - activeBuffer;
            sampleIdx1 = 0; sampleIdx2 = 0;
        }
    }
    portEXIT_CRITICAL_ISR(&timerMux);
    return true;
}

void procesarBuffer(const uint16_t localSamples[], const uint16_t localSamples2[], const uint32_t localTimes[], int n) {
  if (n <= 0) return;
  uint32_t procStart = micros();

  // 1. Obtener Offset DC
  long sum1 = 0, sum2 = 0;
  for(int i=0; i<n; i++) { sum1 += localSamples[i]; sum2 += localSamples2[i]; }
  float avgBits1 = (float)sum1 / n;
  float avgBits2 = (float)sum2 / n;

  float offset1_Teorico = (avgBits1 / 4095.0f) * 3300.0f;
  float offset2_Teorico = (avgBits2 / 4095.0f) * 3300.0f;

  float offset1_Real = obtenerVoltajeReal(offset1_Teorico);
  float offset2_Real = obtenerVoltajeReal(offset2_Teorico);

  ema_offset_trim_1 = alphaEMA * offset1_Real + (1 - alphaEMA) * ema_offset_trim_1;
  ema_offset_trim_2 = alphaEMA * offset2_Real + (1 - alphaEMA) * ema_offset_trim_2;

  // 2. Procesar Muestras
  float vP1_max_inst = 0.0f;
  float vP2_max_inst = 0.0f;

  for (int i = 0; i < n; i++) {
      float valTeorico1 = ((float)localSamples[i] / 4095.0f) * 3300.0f;
      float valTeorico2 = ((float)localSamples2[i] / 4095.0f) * 3300.0f;

      float valReal1 = obtenerVoltajeReal(valTeorico1);
      float valReal2 = obtenerVoltajeReal(valTeorico2);

      float vAC1 = fabs(valReal1 - offset1_Real);
      float vAC2 = fabs(valReal2 - offset2_Real);

      if (vAC1 < 10.0f) vAC1 = 0; 
      if (vAC2 < 10.0f) vAC2 = 0;

      if (vAC1 > vP1_max_inst) vP1_max_inst = vAC1;
      if (vAC2 > vP2_max_inst) vP2_max_inst = vAC2;
  }

  // 3. Filtro Adaptativo
  float alpha1 = (fabs(vP1_max_inst - vPico1_EMA) > UMBRAL_REACCION) ? ALPHA_RAPIDO : ALPHA_LENTO;
  vPico1_EMA = (alpha1 * vP1_max_inst) + (1.0f - alpha1) * vPico1_EMA;

  float alpha2 = (fabs(vP2_max_inst - vPico2_EMA) > UMBRAL_REACCION) ? ALPHA_RAPIDO : ALPHA_LENTO;
  vPico2_EMA = (alpha2 * vP2_max_inst) + (1.0f - alpha2) * vPico2_EMA;

  // 4. Cálculos Finales
  float I_pico1 = vPico1_EMA / SENSOR_SENS1;
  float I_rms1 = I_pico1 / 1.4142f;
  float I_pico2 = vPico2_EMA / SENSOR_SENS2;
  float I_rms2 = I_pico2 / 1.4142f;

  float diferenciaRMS = fabs(I_rms1 - I_rms2);
  float I_rms_promedio_sencillo = ((I_rms1 + I_rms2) / 2.0f);

  I_rms_promedio = I_rms_promedio_sencillo - (K_CORRECCION * diferenciaRMS);
  if (I_rms_promedio < 0.2f) I_rms_promedio = 0; 

  I_pico_promedio = ((I_pico1 + I_pico2) / 2.0f) - (K_CORRECCION * fabs(I_pico1 - I_pico2));
  if (I_pico_promedio < 0.2f) I_pico_promedio = 0;

  // 5. Tiempos
  tAdq_us = (uint32_t)(MAX_SAMPLES * TARGET_DT);
  tProc_us = micros() - procStart;
  tTotal_us = tAdq_us + tProc_us;
  ciclosProcesados++;

  // --- IMPRESIONES DE DEBUG RESTAURADAS ---
  Serial.print("Vp1: ");  Serial.print(vPico1_EMA, 3);
  Serial.print("  Vp2: "); Serial.print(vPico2_EMA, 3);
  Serial.print("  Vp1_inst: ");  Serial.print(vP1_max_inst, 3);
  
  Serial.print("  Off_1: ");  Serial.print(ema_offset_trim_1, 3);
  Serial.print("  Off_2: ");  Serial.print(ema_offset_trim_2, 3);
  
  Serial.print("  Irms1: "); Serial.print(I_rms1, 3);
  Serial.print("  Irms2: "); Serial.print(I_rms2, 3);
  Serial.print("  Diferencia: "); Serial.print(diferenciaRMS, 3);
  Serial.print("  Irms_prom: "); Serial.print(I_rms_promedio, 3);
  
  Serial.print("  Tproc(us): "); Serial.println(tProc_us);
}

// ==========================================
// 6. TAREA PRINCIPAL (CORE 1)
// ==========================================
void TaskProcesar(void *pvParameters) {
  for (;;) {
    int buf = -1;
    portENTER_CRITICAL(&timerMux);
    for (int b = 0; b < NUM_BUFFERS; b++) {
      if (bufferListo[b]) {
        buf = b; bufferListo[b] = false; break;
      }
    }
    portEXIT_CRITICAL(&timerMux);

    if (buf >= 0) {
      uint16_t local1[MAX_SAMPLES], local2[MAX_SAMPLES];
      uint32_t localT[MAX_SAMPLES];
      memcpy(local1, (const void*)samples_mV[buf], sizeof(local1));
      memcpy(local2, (const void*)samples2_mV[buf], sizeof(local2));
      memcpy(localT, (const void*)bufferMicros[buf], sizeof(localT));
      
      procesarBuffer(local1, local2, localT, MAX_SAMPLES);

      int nivelTeorico = 0;
      if (I_rms_promedio > UMBRALES[2])      nivelTeorico = 3;
      else if (I_rms_promedio > UMBRALES[1]) nivelTeorico = 2;
      else if (I_rms_promedio > UMBRALES[0]) nivelTeorico = 1;

      // --- LÓGICA DE TRANSICIÓN ---
      // CASO A: SUBIDA (Falla empeorando)
      if (nivelTeorico > nivelActual) {
          for (int n = nivelActual + 1; n <= nivelTeorico; n++) {
              uint8_t codigoEvento = BASE_SUBIDA + n;
              registrarEventoBinario(codigoEvento, I_rms_promedio);
          }
          nivelActual = nivelTeorico; 
          enSC = true;
      } 
      // CASO B: BAJADA (Recuperación)
      else if (nivelTeorico < nivelActual) {
          float puntoDeCaida = UMBRALES[nivelActual - 1] - HISTeresis;
          if (I_rms_promedio < puntoDeCaida) {
              int nivelAnterior = nivelActual;
              uint8_t codigoEvento = BASE_BAJADA + nivelAnterior;
              registrarEventoBinario(codigoEvento, I_rms_promedio);
              
              nivelActual--; 
              
              if (nivelActual == 0) {
                  enSC = false;
                  Serial.println("\n[SISTEMA] Regreso a condiciones normales.");

                  imprimirMemoriaRaw();
              }
          }
      }
    }

    // Heartbeat cada 1 segundo (Resumen)
    unsigned long tic = millis();
    if (tic - ultimoConteo >= 1000) {
      Serial.printf(">>> HEARTBEAT: RMS: %.2f A | Nivel: %d | Ciclos/s: %lu | %s\n", 
                    I_rms_promedio, nivelActual, ciclosProcesados, getTimestamp().c_str());
      ciclosProcesados = 0; ultimoConteo = tic;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ==========================================
// 7. SETUP
// ==========================================
void setup() {
  Serial.begin(500000); 
  delay(500);
  Serial.println("\n=== INICIANDO SISTEMA V5 REPARADO (DEBUG ACTIVO) ===");

  // Configuración Hardware ADC
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);
  analogSetPinAttenuation(SENSOR_PIN_2, ADC_11db);

  // Configuración DMA
  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = (size_t)(MAX_SAMPLES * 2 * sizeof(adc_digi_output_data_t)),
    .conv_frame_size = (MAX_SAMPLES / 2) * sizeof(adc_digi_output_data_t)
  };

  if (adc_continuous_new_handle(&handle_cfg, &my_adc_handle) == ESP_OK) {
    adc_digi_pattern_config_t pattern[2] = {0};
    
    // Canal 1 (Pin 4)
    pattern[0].atten = ADC_ATTEN_DB_11; 
    pattern[0].channel = ADC_CHANNEL_3; 
    pattern[0].unit = ADC_UNIT_1; 
    pattern[0].bit_width = ADC_BITWIDTH_12;
    
    // Canal 2 (Pin 7)
    pattern[1].atten = ADC_ATTEN_DB_11; 
    pattern[1].channel = ADC_CHANNEL_6; 
    pattern[1].unit = ADC_UNIT_1; 
    pattern[1].bit_width = ADC_BITWIDTH_12;
    
    adc_continuous_config_t dig_cfg = {0};
    dig_cfg.pattern_num = 2; 
    dig_cfg.adc_pattern = pattern;
    
    uint32_t perChannelHz = (TARGET_DT > 0) ? (1000000UL / TARGET_DT) : 4000;
    dig_cfg.sample_freq_hz = perChannelHz * 2; 
    dig_cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1; 
    dig_cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    
    if (adc_continuous_config(my_adc_handle, &dig_cfg) == ESP_OK) {
      adc_continuous_evt_cbs_t cbs = { .on_conv_done = adc_dma_isr };
      adc_continuous_register_event_callbacks(my_adc_handle, &cbs, NULL);
      adc_continuous_start(my_adc_handle);
      Serial.println("-> ADC DMA iniciado correctamente.");
    } else {
      Serial.println("Error configurando ADC DMA");
    }
  } else {
     Serial.println("Error creando handle ADC");
  }

  setupWiFi(); 

  // Crear Tarea en Core 1
  xTaskCreatePinnedToCore(
    TaskProcesar,    
    "ProcesarADC",   
    10000,           
    NULL,            
    1,               
    &taskProcesarHandle, 
    1                
  );

  Serial.println("-> Tarea de procesamiento iniciada.");
}

void loop() {
    // Vacío, todo en TaskProcesar
}