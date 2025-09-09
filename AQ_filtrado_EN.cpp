#include <Wire.h>
#include <math.h>

// ---------- Configurações de pinos I2C no ESP32 ----------
#define I2C_SDA 21
#define I2C_SCL 22

// ---------- Endereço I2C do MPU-6050 ----------
const uint8_t MPU_ADDR = 0x68; // AD0=GND -> 0x68 ; AD0=VCC -> 0x69

// ---------- Parâmetros de amostragem e filtro ----------
const unsigned long SAMPLE_INTERVAL_MS = 100; // 10 Hz (>= seu mínimo)
const float LPF_ALPHA = 0.85f; // filtro passa-baixa (0.0 = sem filtro, 0.85 = filtra bem)

// ---------- Parâmetros de detecção (calibráveis) ----------
const float ACC_THRESHOLD_G = 2.2f;      // pico de impacto (em g). Teste 2.5 ~ 3.0 g
const float ANGLE_DELTA_DEG = 50.0f;     // mudança de orientação em até 1s
const unsigned long WINDOW_MS = 1000;    // janela para avaliar variação angular
const unsigned long DEBOUNCE_MS = 3000;  // evita alertas em sequência

// ---------- Estado global ----------
unsigned long lastSample = 0;
unsigned long lastAlert  = 0;

// Leituras brutas
int16_t ax_r, ay_r, az_r, temp_r, gx_r, gy_r, gz_r;

// Aceleração em g (filtrada)
float ax_f = 0, ay_f = 0, az_f = 0;

// Para janela de ângulo (referência = sqrt(pitch^2 + roll^2))
const int BUF_CAP = (WINDOW_MS / SAMPLE_INTERVAL_MS) + 4;
float angleBuf[BUF_CAP];
unsigned long timeBuf[BUF_CAP];
int bufSize = 0;
int bufHead = 0;

// Utilitários
static inline float rad2deg(float r){ return r * 57.2957795f; }
static inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v); }

bool mpuWrite(uint8_t reg, uint8_t val){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readMPUAll(){
  // Lê 14 bytes a partir de ACCEL_XOUT_H
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true) != 14) return false;

  ax_r   = (Wire.read() << 8) | Wire.read(); // ACCEL_XOUT_H/L
  ay_r   = (Wire.read() << 8) | Wire.read(); // ACCEL_YOUT_H/L
  az_r   = (Wire.read() << 8) | Wire.read(); // ACCEL_ZOUT_H/L
  temp_r = (Wire.read() << 8) | Wire.read(); // TEMP_OUT_H/L  (não usamos aqui)
  gx_r   = (Wire.read() << 8) | Wire.read(); // GYRO_XOUT_H/L
  gy_r   = (Wire.read() << 8) | Wire.read(); // GYRO_YOUT_H/L
  gz_r   = (Wire.read() << 8) | Wire.read(); // GYRO_ZOUT_H/L
  return true;
}

void pushAngle(float a, unsigned long t){
  angleBuf[bufHead] = a;
  timeBuf[bufHead]  = t;
  bufHead = (bufHead + 1) % BUF_CAP;
  if (bufSize < BUF_CAP) bufSize++;
}

void getWindowMinMax(unsigned long now, float &amin, float &amax){
  amin =  1e9f;
  amax = -1e9f;
  bool any = false;
  for (int i = 0; i < bufSize; i++){
    int idx = (bufHead - 1 - i + BUF_CAP) % BUF_CAP;
    if (now - timeBuf[idx] <= WINDOW_MS){
      float v = angleBuf[idx];
      if (v < amin) amin = v;
      if (v > amax) amax = v;
      any = true;
    } else break;
  }
  if (!any){ amin = 0.f; amax = 0.f; }
}

void setup(){
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // I2C em 400 kHz (rápido e estável)

  // Acorda o MPU-6050
  if (!mpuWrite(0x6B, 0x00)){ // PWR_MGMT_1 = 0 (wake)
    Serial.println("Falha ao acordar MPU-6050 (I2C). Cheque fiação.");
  }

  // Configura faixas padrão: AFS_SEL=±2g (0x1C=0x00), FS_SEL=±250 dps (0x1B=0x00)
  mpuWrite(0x1C, 0x00); // ACCEL_CONFIG
  mpuWrite(0x1B, 0x00); // GYRO_CONFIG

  Serial.println("MPU-6050 pronto. Iniciando leituras...");
}

void loop(){
  unsigned long now = millis();
  if (now - lastSample < SAMPLE_INTERVAL_MS) return;
  float dt = (now - lastSample) / 1000.0f;
  lastSample = now;

  if (!readMPUAll()){
    Serial.println("Leitura I2C falhou.");
    return;
  }

  // Conversões para unidades físicas
  // AFS_SEL=±2g => 16384 LSB/g
  // FS_SEL=±250 dps => 131 LSB/(°/s)
  float ax = (float)ax_r / 16384.0f;
  float ay = (float)ay_r / 16384.0f;
  float az = (float)az_r / 16384.0f;

  float gx = (float)gx_r / 131.0f; // °/s (não usado na regra atual, mas já disponível)
  float gy = (float)gy_r / 131.0f;
  float gz = (float)gz_r / 131.0f;

  // Filtro passa-baixa (EMA) nas acelerações para estabilizar orientação
  if (ax_f == 0 && ay_f == 0 && az_f == 0){
    ax_f = ax; ay_f = ay; az_f = az; // inicializa filtro na 1ª leitura
  } else {
    ax_f = LPF_ALPHA * ax_f + (1.0f - LPF_ALPHA) * ax;
    ay_f = LPF_ALPHA * ay_f + (1.0f - LPF_ALPHA) * ay;
    az_f = LPF_ALPHA * az_f + (1.0f - LPF_ALPHA) * az;
  }

  // Magnitude de aceleração (em g) sem filtrar para capturar picos de impacto
  float aMagG = sqrtf(ax*ax + ay*ay + az*az);

  // Ângulos de inclinação (a partir do vetor acelerômetro filtrado)
  float pitch_acc = rad2deg(atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)));
  float roll_acc  = rad2deg(atan2f( ay_f, az_f));
  float angRef    = sqrtf(pitch_acc*pitch_acc + roll_acc*roll_acc);

  // Atualiza janela de 1s e calcula delta de ângulo
  pushAngle(angRef, now);
  float amin, amax;
  getWindowMinMax(now, amin, amax);
  float angleDelta = amax - amin;

  // Regra de detecção (impacto + mudança rápida de orientação)
  bool queda = false;
  if (aMagG >= ACC_THRESHOLD_G && angleDelta >= ANGLE_DELTA_DEG){
    if (now - lastAlert > DEBOUNCE_MS){
      queda = true;
      lastAlert = now;
    }
  }

  // Logs compactos (ajuste como preferir)
  Serial.print("aMagG=");
  Serial.print(aMagG, 2);
  Serial.print(" | pitch=");
  Serial.print(pitch_acc, 1);
  Serial.print(" roll=");
  Serial.print(roll_acc, 1);
  Serial.print(" | dAng=");
  Serial.print(angleDelta, 1);

  if (queda) Serial.println("  => QUEDA DETECTADA!");
  else       Serial.println("  => OK");
}
