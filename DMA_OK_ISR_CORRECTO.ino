#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc/adc_continuous.h"

// ===== CONFIG ADC =====
const int SENSOR_PIN = 4;
const int SENSOR_PIN_2 = 7;
const int MAX_SAMPLES = 64;
const unsigned long CYCLE_US = 16666UL;  // 1 ciclo de 60 Hz
const unsigned long TARGET_DT = CYCLE_US / MAX_SAMPLES; // ~260 µs entre muestras

// Ganancia y sensibilidad
const float SENSOR_SENS1 = 12.43984f;
const float SENSOR_SENS2 = 12.47566f;

// ===== Buffers tipo "ping-pong" =====
#define NUM_BUFFERS 2
volatile uint16_t samples_mV[NUM_BUFFERS][MAX_SAMPLES];
volatile uint16_t samples2_mV[NUM_BUFFERS][MAX_SAMPLES];
volatile uint32_t bufferMicros[NUM_BUFFERS][MAX_SAMPLES];
volatile int activeBuffer = 0;
volatile int sampleIdx1 = 0;
volatile int sampleIdx2 = 0;
volatile bool bufferListo[NUM_BUFFERS] = {false, false};

// ===== FreeRTOS =====
TaskHandle_t taskProcesarHandle = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ===== Resultados =====
float I_pico_promedio = 0.0f;
float I_rms_promedio = 0.0f;
uint32_t tAdq_us = 0, tProc_us = 0, tTotal_us = 0;
volatile unsigned long ciclosProcesados = 0;
unsigned long ultimoConteo = 0;

// ===== EMA (filtro offset) =====
float ema_offset_trim_1 = 0.0f, ema_offset_trim_2 = 0.0f;
const float alphaEMA = 0.20f;

// ===== Sobrecorriente =====
const float UMBRAL_SC = 5.0f;
const uint32_t INTERVALO_ACTUALIZACION = 15000; 

struct EventoSC {
  String estado;
  float pico;
  uint32_t inicio;
  uint32_t duracion;
};

EventoSC historico[50];
int indiceHist = 0;
int totalEventos = 0;

bool enSC = false;
uint32_t tiempoInicioSC = 0;
uint32_t tiempoUltimoRegistro = 0;
float picoEvento = 0.0;
String estadoSobrecorriente = "Inactivo";
uint32_t duracionSC = 0;

float K_CORRECCION = 0.28f; 

// ===== Utilidades =====
inline uint32_t deltaMicros(uint32_t start, uint32_t end) {
  return (end >= start) ? (end - start) : (UINT32_MAX - start + end + 1);
}

float trimmedMeanNoSort(const uint16_t arr[], int n, int trimCount) {
  if (n <= 2 * trimCount) {
    long s = 0;
    for (int i = 0; i < n; i++) s += arr[i];
    return (float)s / n;
  }
  bool removed[MAX_SAMPLES] = {0};
  long sum = 0;
  for (int i = 0; i < n; i++) sum += arr[i];
  int remaining = n;

  for (int t = 0; t < trimCount; t++) {
    int minIdx = -1, maxIdx = -1;
    int minVal = INT_MAX, maxVal = INT_MIN;
    for (int i = 0; i < n; i++) {
      if (removed[i]) continue;
      int v = arr[i];
      if (v < minVal) { minVal = v; minIdx = i; }
      if (v > maxVal) { maxVal = v; maxIdx = i; }
    }
    if (minIdx >= 0) { removed[minIdx] = true; sum -= minVal; remaining--; }
    if (maxIdx >= 0) { removed[maxIdx] = true; sum -= maxVal; remaining--; }
  }
  return (remaining > 0) ? (float)sum / remaining : 0.0f;
}

// ===== ADC continuous (DMA) setup =====
#define ADC_GPIO4_CHANNEL ADC_CHANNEL_3  
#define ADC_GPIO7_CHANNEL ADC_CHANNEL_6  

adc_continuous_handle_t my_adc_handle = NULL;
const size_t DMA_BUFFER_SIZE = MAX_SAMPLES * 2 * sizeof(adc_digi_output_data_t);

// Callback del ADC DMA (CORREGIDO)
static bool IRAM_ATTR adc_dma_isr(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    // Acceso directo a los datos entregados por el driver
    size_t entries = edata->size / sizeof(adc_digi_output_data_t);
    adc_digi_output_data_t *p = (adc_digi_output_data_t *)edata->conv_frame_buffer;

    portENTER_CRITICAL_ISR(&timerMux);
    for (size_t i = 0; i < entries; i++) {
        uint8_t ch = p[i].type2.channel;
        int val = p[i].type2.data;  
        uint32_t now = micros();

        if (ch == ADC_GPIO4_CHANNEL && sampleIdx1 < MAX_SAMPLES) {
            samples_mV[activeBuffer][sampleIdx1] = val;
            bufferMicros[activeBuffer][sampleIdx1] = now;
            sampleIdx1++;
        }
        else if (ch == ADC_GPIO7_CHANNEL && sampleIdx2 < MAX_SAMPLES) {
            samples2_mV[activeBuffer][sampleIdx2] = val;
            bufferMicros[activeBuffer][sampleIdx2] = now;
            sampleIdx2++;
        }

        if (sampleIdx1 >= MAX_SAMPLES && sampleIdx2 >= MAX_SAMPLES) {
            bufferListo[activeBuffer] = true;
            activeBuffer = 1 - activeBuffer;
            sampleIdx1 = 0;
            sampleIdx2 = 0;
        }
    }
    portEXIT_CRITICAL_ISR(&timerMux);

    return true;
}

// ===== Procesamiento =====
void procesarBuffer(const uint16_t localSamples[], const uint16_t localSamples2[], const uint32_t localTimes[], int n) {
  if (n <= 0) return;
  uint32_t procStart = micros();

  int trimCount = n / 10;
  float offset1 = trimmedMeanNoSort(localSamples, n, trimCount);
  float offset2 = trimmedMeanNoSort(localSamples2, n, trimCount);

  ema_offset_trim_1 = alphaEMA * offset1 + (1 - alphaEMA) * ema_offset_trim_1;
  ema_offset_trim_2 = alphaEMA * offset2 + (1 - alphaEMA) * ema_offset_trim_2;

  const float ADC_TO_MV = 3300.0f / 4095.0f;
  float offset1_mV_reporte = ema_offset_trim_1 * ADC_TO_MV;
  float offset2_mV_reporte = ema_offset_trim_2 * ADC_TO_MV;
  
  float vPico1 = 0.0f, vPico2 = 0.0f;

  for (int i = 0; i < n; i++) {
      int16_t v1_crudo = localSamples[i] - (int)ema_offset_trim_1;
      int16_t v2_crudo = localSamples2[i] - (int)ema_offset_trim_2;
      int16_t v1_abs = abs(v1_crudo);
      int16_t v2_abs = abs(v2_crudo);
      if (v1_abs > vPico1) vPico1 = v1_abs;
      if (v2_abs > vPico2) vPico2 = v2_abs;
  }

  float FACTOR_CALIBRACION_V_PICO = 0.96f;
  float vPico1_mV = vPico1 * ADC_TO_MV * FACTOR_CALIBRACION_V_PICO;
  float vPico2_mV = vPico2 * ADC_TO_MV * FACTOR_CALIBRACION_V_PICO;

  float I_pico1 = vPico1_mV / SENSOR_SENS1;
  float I_rms1 = I_pico1 / 1.4142f;
  float I_pico2 = vPico2_mV / SENSOR_SENS2;
  float I_rms2 = I_pico2 / 1.4142f;

  float diferenciaRMS = fabs(I_rms1 - I_rms2);
  float I_rms_promedio_sencillo = ((I_rms1 + I_rms2) / 2.0f);

  I_rms_promedio = ((I_rms1 + I_rms2) / 2.0f) - (K_CORRECCION * diferenciaRMS);
  if (I_rms_promedio < 0) I_rms_promedio = 0;

  I_pico_promedio = ((I_pico1 + I_pico2) / 2.0f) - (K_CORRECCION * fabs(I_pico1 - I_pico2));
  if (I_pico_promedio < 0) I_pico_promedio = 0;

  tAdq_us = (uint32_t)(MAX_SAMPLES * TARGET_DT);
  tProc_us = micros() - procStart;
  tTotal_us = tAdq_us + tProc_us;

  ciclosProcesados++;
  Serial.print("Vp1: ");  Serial.print(vPico1_mV, 3);
  Serial.print("  Vp2: "); Serial.print(vPico2_mV, 3);
  Serial.print("  Off_1: ");  Serial.print(offset1_mV_reporte, 3);
  Serial.print("   Off_2: ");  Serial.print(offset2_mV_reporte, 3);
  Serial.print("  Irms1: "); Serial.print(I_rms1, 3);
  Serial.print("  Irms2: "); Serial.print(I_rms2, 3);
  Serial.print("  Diferencia: "); Serial.print(diferenciaRMS, 3);
  Serial.print("  Irms_prom_Factor: "); Serial.print(I_rms_promedio, 3);
  Serial.print("  Irms_prom: "); Serial.print(I_rms_promedio_sencillo, 3);
  Serial.print("  Tadq(us): "); Serial.print(tAdq_us);
  Serial.print("  Tproc(us): "); Serial.print(tProc_us);
  Serial.print("  Ttotal(us): "); Serial.println(tTotal_us);
}

void imprimirHistorico() {
  Serial.println("\n===== HISTÓRICO DE EVENTOS =====");
  for (int i = 0; i < totalEventos; i++) {
    Serial.printf("%2d) %-10s | Pico: %.3f A | Inicio: %lu ms | Duración: %lu ms\n",
                  i + 1, historico[i].estado.c_str(),
                  historico[i].pico, historico[i].inicio, historico[i].duracion);
  }
  Serial.println("===============================");
}

void TaskProcesar(void *pvParameters) {
  static uint32_t ultimoReporteSC = 0;

  for (;;) {
    int buf = -1;
    portENTER_CRITICAL(&timerMux);
    for (int b = 0; b < NUM_BUFFERS; b++) {
      if (bufferListo[b]) {
        buf = b;
        bufferListo[b] = false;
        break;
      }
    }
    portEXIT_CRITICAL(&timerMux);

    if (buf >= 0) {
      uint16_t local1[MAX_SAMPLES];
      uint16_t local2[MAX_SAMPLES];
      uint32_t localT[MAX_SAMPLES];
      for (int i = 0; i < MAX_SAMPLES; i++) {
        local1[i] = samples_mV[buf][i];
        local2[i] = samples2_mV[buf][i];
        localT[i] = bufferMicros[buf][i];
      }
      procesarBuffer(local1, local2, localT, MAX_SAMPLES);
    } else {
      vTaskDelay(1);
    }

    unsigned long ahora = millis();

    if (enSC) {
      duracionSC = ahora - tiempoInicioSC;
      if (estadoSobrecorriente == "Inicio" || (ahora - ultimoReporteSC >= INTERVALO_ACTUALIZACION)) {
        ultimoReporteSC = ahora;
        estadoSobrecorriente = "EnProceso";
        Serial.printf("[SC] Estado: %-10s | Pico actual: %.3f A | Duración: %lu ms | Inicio: %lu ms\n",
                      estadoSobrecorriente.c_str(), picoEvento, duracionSC, tiempoInicioSC);
      }
    } else if (estadoSobrecorriente == "Fin") {
      Serial.printf("[SC] Estado: %-10s | Pico final: %.3f A | Duración: %lu ms | Inicio: %lu ms\n",
                    estadoSobrecorriente.c_str(), picoEvento, duracionSC, tiempoInicioSC);
      imprimirHistorico();
      estadoSobrecorriente = "Inactivo";
    }

    if (ahora - ultimoConteo >= 1000) {
      ultimoConteo = ahora;
      Serial.printf("Ciclos/seg: %lu (esperado ~60)\n", ciclosProcesados);
      if (!enSC) Serial.println("Estado: Inactivo");
      ciclosProcesados = 0;
    }
  }
}

void setup() {
  Serial.begin(500000);
  delay(200);

  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);
  analogSetPinAttenuation(SENSOR_PIN_2, ADC_11db);

  adc_continuous_handle_cfg_t handle_cfg = {
    .max_store_buf_size = DMA_BUFFER_SIZE,
    // Frame size pequeño para procesar datos conforme llegan sin latencia
    .conv_frame_size = (MAX_SAMPLES / 2) * sizeof(adc_digi_output_data_t)
  };

  if (adc_continuous_new_handle(&handle_cfg, &my_adc_handle) == ESP_OK) {
    adc_digi_pattern_config_t pattern[2] = {0};
    pattern[0].atten = ADC_ATTEN_DB_11;
    pattern[0].channel = ADC_GPIO4_CHANNEL;
    pattern[0].unit = ADC_UNIT_1;
    pattern[0].bit_width = ADC_BITWIDTH_12;

    pattern[1].atten = ADC_ATTEN_DB_11;
    pattern[1].channel = ADC_GPIO7_CHANNEL;
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
      Serial.println("ADC DMA iniciado.");
    }
  }

  xTaskCreatePinnedToCore(TaskProcesar, "TaskProcesar", 8192, NULL, 3, &taskProcesarHandle, 0);
  Serial.println("Sistema listo.");
}

void loop() {}