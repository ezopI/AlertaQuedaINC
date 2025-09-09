#include <math.h> // Biblioteca de cálculos matemáticos
#include <Wire.h> // Biblioteca da comunicação entre fios

//--- Configuração dos pinos SDA21 e SCL22 na comuniação I2C ---
#define I2C_SDA 21
#define I2C_SCL 22

//--- Configuração do endereço do MPU6050 ---
const uint8_t MPU_ENDR = 0x68; //endereço padrão

//--- Parâmetros de amostragem e filtro de leitura ---
const unsigned long AMOSTRA_INTERVALO_MS = 100; // 10Hz (10 amostras por segundo (1000/100))
const float LPF_ALPHA = 0.85f; // filtro Low-Pass(para redução de ruídos nas leituras(0.0 = sem filtro, 0.85 = filtra bem))

//--- Parâmetros de detecção ---
const float ACL_LIMITE_G = 2.2f;            // aceleração do pico de impacto (em g). Teste 2.5 ~ 3.0 g
const float ANGULO_VARIACAO_GRAU = 50.0f;   // mudança de orientação em graus em até 1s. O limite para variar é de 50°, a partir disso é um indicativo de queda
const unsigned long JANELA_MS = 1000;       // janela para analisar a viaração angular em 1s. 1000ms == 1s
const unsigned long PAUSA_MS = 2000;        // evita diversos alertas em sequência(TERMINAL)

//--- Estado global ---
unsigned long ultimoMomento = 0;    // fás a leitura do último momento ocorrido para poder faer comparações (valor é agregado a partir do AMOSTRA_INTERVALO_MS)
unsigned long ultimoAlerta = 0;     // registra a última queda detectada e é usado junto com PAUSA_MS para não fasser diversos disparos. É a "memória" da última queda.

// Leituras brutas
int16_t ax_c, ay_c, az_c, gx_c, gy_c, gz_c, temp_c; //dividindo-os por 16384.0 para se transformar em (m/s², g ou °/s)

// Aceleração em g(filtrado)
float ax_f = 0, ay_f = 0, az_f = 0;

// Janela de observação do ângulo (referência = sqrt())


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
