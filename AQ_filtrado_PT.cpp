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
unsigned long ultimoMomento = 0;    // faz a leitura do último momento ocorrido para poder fazer comparações (valor é agregado a partir do AMOSTRA_INTERVALO_MS)
unsigned long ultimoAlerta = 0;     // registra a última queda detectada e é usado junto com PAUSA_MS para não fazer diversos disparos. É a "memória" da última queda.

// Leituras brutas
int16_t ax_c, ay_c, az_c, gx_c, gy_c, gz_c, temp_c; // dividindo-os por 16384.0 para se transformar em (m/s², g ou °/s)

// Aceleração em g(filtrado)
float ax_f = 0, ay_f = 0, az_f = 0;

// Janela de observação do ângulo (referência = sqrt())
const int BUF_CAP = (JANELA_MS / AMOSTRA_INTERVALO_MS + 4); // Buffer é o espaço de memória temporário de uma variável. No calculo mostra a quantidade de amostras que cabem na janela de tempo e o +4 é uma margem de segurança
float angleBuf[BUF_CAP];                                    // vetor que guarda os ângulos calculados (pitch/roll ou magnetude de inclinação)
unsigned long tempoBuf[BUF_CAP];                            // vetor que guarda o tempo em cada ângulo medido
int bufTamanho = 0;                                         // quantidade de elementos válidos no Buffer
int bufTopo = 0;                                            // índice do próximo lugar onde será inserida uma nova medida (funciona como buffer circular: quando chega no fim, volta para o começo).

// Utilitários
static inline float rad2deg(float r){ return r * 57.2957795f; }       // transforma os número de radianos para graus, pois as bibliotecas do arduino usam em radianos
static inline float clampf(float v, float lo, float hi){ return v < lo ? lo : (v > hi ? hi : v);}       // protege contra erros numéricos (NaN) e busca arredondar os valores para interpretação

bool mpuIniciar(uint8_t reg, uint8_t val){
  Wire.beginTrasnmission(MPU_ENDR);
  Wire.write(reg); // endereço do registrador interno (0x6B), lugar que vamos acessar
  Wire.write(val); // valor que vamos gravar no registrador
  return Wire.endTransmission() == 0;
}

bool leituraMPUTudo(){
  // Lê 14 bytes a partir de ACCEL_XOUT_H
  Wire.beginTrasnmission(MPU_ENDR);                   // inicia a comunicação com o MPU
  Wire.write(0x3B);                                   // 0x3B é o endereço interno do registrador ACCEL_XOUT_H()
  if (Wire.endTransmission(false) != 0) return false; //encerra a transmissão, porém, permanece com o barramento ativo
  if (wire.requestFrom(MPU_ENDR, (uint8_t)14, (uint8_t)true) != 14) return false; // pede 14 bytes do endereço 0x3B(são diversas leituras). Se uma falhar, retorna falso.

  ax_r   = (Wire.read() << 8) | Wire.read(); // ACCEL_XOUT_H/L      --- MPU envia a cada dado 2 bytes(high+low)
  ay_r   = (Wire.read() << 8) | Wire.read(); // ACCEL_YOUT_H/L
  az_r   = (Wire.read() << 8) | Wire.read(); // ACCEL_ZOUT_H/L
  gx_r   = (Wire.read() << 8) | Wire.read(); // GYRO_XOUT_H/L
  gy_r   = (Wire.read() << 8) | Wire.read(); // GYRO_YOUT_H/L
  gz_r   = (Wire.read() << 8) | Wire.read(); // GYRO_ZOUT_H/L
  return true;
}

void anguloImpulso(float a, unsigned long t){
  anguloBuf[bufTopo] = a;     // guarda o valor do ângulo
  tempoBuf[bufTopo] = t;      // guarda o tempo correspondente
  bufTopo = (bufTopo + 1) % BUF_CAP;      // é o que torna o Buffer circular, pois toda vez que bater o máximo do buffer ele retornará a 0
  if (bufTamanho < BUF_CAP) bufTamanho++; // Conta quantos valores há no buffer, depois que bate o limite(BUF_CAP) ele sobrescreve para não passar do máximo
}

void receberJanelaMinMax(unsigned long agora, float &amin, float &amax){
  amin = 1e9f;        // valor muito grande 1B
  amax = -1e9f;       //valor muito pequeno -1B   valores reais lidos do buffer
  bool nada = false;  // serve para marcar se algum valor válido foi encontrado, senão é zerado.
  for (int i = 0; i < bufTamanho; i++){                   // percorre valores que já existem no buffer
    int idx = (bufTopo - 1 - i + BUF_CAP) % BUF_CAP;      // vai captando de trás para a frente os dados
    if (agora - tempoBuf[idx] <= JANELA_MS){              // calcula o tempo que se passou desde a medição (aceita valores apenas dentro da janela de tempo definida(JANELA_MS))
      float v = anguloBuf[idx];     // pega o valor do angulo no instante
      if (v < amin) amin = v;       // compara
      if (v > amax) amax = v;       // compara
      nada = true;                  // então, atualiza se encontrou um valor válido
    }  else break;                  // se encontrar um valor antigo, ele para
  }
  if (!nada){ amin = 0.f; amax = 0.f; } // se não encontrou nenhum valor, retornar amin e amax para 0
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000)   // I2C em 400kHz (rápido e estável)

  // Inicia o MPU-6050
  if (!mpuIniciar(0x6B, 0x00)){ //PWR_MGMT_1 = 0 (LEVANTAR)
    Serial.println("Falha ao acordar o MPU-6050 (I2C). Cheque a fiação");
  }

  // Configura faixas padrão: AFS_SEL =±2g (0x1C=0x00), FS_SEL=±250 dps (0x1B=0x00)
  mpuIniciar(0x1C, 0x00); // ACCEL_CONFIG
  mpuIniciar(0x1B, 0x00); // GYRO_CONFIG

  Serial.println("MPU-6050 pronto. Iniciando leituras...");
}

void loop() {
  unsigned long agora = millis();                             // retorna o tempo em MS
  if (agora - ultimoMomento < AMOSTRA_INTERVALO_MS) return;   // mede o tempo que passou, caso não tenha lindo nada ele sai sem acontecer nada.
  float dt = (agora - ultimoMomento) / 1000.0f;               // converte a diferença de tempo para segundos. "dt" pode ser usados para cálculos que dependem de tmepo.
  ultimoMomento = agora;                                      // torna o último momento como agora

  if(!leituraMPUTudo()){
    Serial.println("Leitura I2C falhou.")
    return;
  }

  // Conversões para unidades físicas
  // AFS_SEL=± => 16384 LSB/g   Last Significant Bit
  // FS_SEL=±250 dps => 131LSB/(°/s)
  float ax = (float)ax_c / 16384.0f;
  float ay = (float)ay_c / 16384.0f;
  float az = (float)az_c / 16384.0f;    // dados crus transformados em g(gravidade). Se az ≈ 1.0 g com ax, ay ≈ 0, significa que o sensor está parado e alinhado com a gravidade.

  float gx = (float)gx_c / 131.0f;
  float gy = (float)gy_c / 131.0f;
  float gz = (float)gz_c / 131.0f;      // dados crus transformados em °/s

  // Filtro passa-baixo(Low-Pass) (EMA) nas acelerações para estabilizar orientação
  if (ax_f == 0 && ay_f == 0 && az_f == 0){
    ax_f = ax; ay_f = ay; az_f = az;      // recebe os valores brutos
  } else {
    ax_f = LPF_ALPHA * ax_f + (1.0f - LPF_ALPHA) * ax;
    ay_f = LPF_ALPHA * ay_f + (1.0f - LPF_ALPHA) * ay;
    az_f = LPF_ALPHA * az_f + (1.0f - LPF_ALPHA) * az;  // filtra o valor de acordo com o ALPHA
  }

  // Magnitude de aceleração em g sem filtrar para capturar picos de impacto
  float aMagG = sqrtf(ax*ax + ay*ay + az*az);

  // Ângulo de inclinação (a partir do vetor acelerômetro filtrado)
  float pitch_acc = rad2deg(atan2f(-ax_f, sqrtf(ay_f*ay_f + az_f*az_f)));   // faz o cálculo do ângulo frente e trás
  float roll_acc  = rad2deg(atan2f(ay_f, az_f));                            // faz o cálculo do ângulo esquerda e direita
  float angRef    = sqrtf(pitch_acc*pitch_acc + roll_acc*roll_acc);         // combina o pitch e o roll em uma única medida de "quanto o dispositivo está inclinado"

  // Atualiza janela de 1s e calcula delta de ângulo
  anguloImpulso(angRef, agora);           // cria um "histórico" de como o corpo se movimento ao longo do tempo
  float amin, amax;                       // VAR para armarzenar o minímo e o máximo
  receberJanelaMinMax(agora, amin, amax); // função que percorre o Buffer circular, permitindo saber quanto a orientação variou no intervalo
  float anguloDelta = amax- amin;         // calcula a diferença entre o maio e o menor ângulo, ou seja, a mudança total de orientação. Se em 1s o corpo estava de pé (20°) e depois foi para deitado (100°) amax - amin = 100 - 20 = 80°. Isso caracteriza uma grande variação, compatível com uma queda.

  // Regra de detecção (impacto + mudança rápida de orientação)
  bool queda = false;     // inicia a queda em falso
  if (aMagG >= ACL_LIMITE_G && anguloDelta >= ANGULO_VARIACAO_GRAU){  // condição que pega a aceleração total em g e compara com o impacto configurado, significando uma colisão forte do corpo. //AND pega a variação de orientação do corpo e detecta a mudança de posição rápida
    if (agora - ultimoAlerta > PAUSA_MS){     // serve para não dar diversos disparos por causa da mesma queda, ou seja, a queda que ocorrer é uma queda forte.
      queda = true;                           // houve a queda
      lastAlert = agora;
    }
  }

  // Respostas compactas
  Serial.print("aMagG=");
  Serial.print(aMagG, 2);
  Serial.print(" | pitch=");
  Serial.print(pitch_acc, 1);
  Serial.print(" roll=");
  Serial.print(roll_acc, 1);
  Serial.print(" | dAng=");
  Serial.print(anguloDelta, 1);

  if (queda)  Serial.println("    => QUEDA DETECTADA!");
  else        Serial.println("    => OK");

}
