#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h>

// Bluetooth
BluetoothSerial SerialBT;

// Servo
Servo myservo;  

// Define pinos
#define LED_PIN 35      // LED azul
#define BUZZER_PIN 14   // Buzzer
#define BUTTON_PIN 21   // Botão
#define ECHO_PIN 25     // ECHO sensor ultrassônico
#define TRIG_PIN 26     // TRIG sensor ultrassônico
#define MEIO_AO_PIN 27  // Sensor de chamas meio
#define ESQ_AO_PIN 15   // Sensor de chamas esquerda
#define DIR_AO_PIN 4    // Sensor de chamas direita
#define SERVO_PIN 22    // Servo
#define in1 33          // Motores
#define in2 32          // Motores
#define in3 13          // Motores
#define in4 2           // Motores
#define spd 5           // Motores
#define BOMBA 12        // Bomba

#define DISTANCE_THRESHOLD 10 // limite de aproximação

// Máquina de estados
#define NUM_ESTADOS 4
#define NUM_EVENTOS 6

// ESTADOS
#define PARADO 0
#define VARREDURA 1
#define DESVIO 2
#define BOMBEIRO 3

// EVENTOS
#define NENHUM_EVENTO -1
#define ON 0
#define OFF 1
#define OBSTACULO 2
#define SEM_OBSTACULO 3
#define CHAMA 4
#define SEM_CHAMA 5

// ACOES
#define NENHUMA_ACAO -1
#define A01 0  // PARADO --> VARREDURA
#define A02 1  // VARREDURA --> DESVIO
#define A03 2  // DESVIO --> VARREDURA
#define A04 3  // VARREDURA --> BOMBEIRO
#define A05 4  // BOMBEIRO --> VARREDURA
#define A06 5  // ESTADOS --> PARADO

// Tasks 
void mov1Task(void *arg);
void desligamentoTask(void *arg);
void apaga_fogoTask(void *arg);
void desvioTask(void *arg);

// FreeRTOS
QueueHandle_t xQueue;
SemaphoreHandle_t xBinarySemaphore1;
SemaphoreHandle_t xBinarySemaphore2;
SemaphoreHandle_t xBinarySemaphore3;
SemaphoreHandle_t xBinarySemaphore4;
TaskHandle_t mov1TaskH;
TaskHandle_t desligamentoTaskH;
TaskHandle_t apaga_fogoTaskH;
TaskHandle_t desvioTaskH;

// Tabelas de Transição
int acao_matrizTransicaoEstados[NUM_ESTADOS][NUM_EVENTOS];
int proximo_estado_matrizTransicaoEstados[NUM_ESTADOS][NUM_EVENTOS];

int executarAcao(int codigoAcao) {
    int retval = NENHUM_EVENTO;

    if (codigoAcao == NENHUMA_ACAO) return retval;

    switch (codigoAcao) {
        case A01: // Robô faz movimento de varredura - mov1Task
            if (xSemaphoreGive(xBinarySemaphore1) != pdPASS)
                Serial.println("Erro ao enviar para semáforo 1");
            break;
        case A02: // Robô desvia de obstáculo - desvioTask
            if (xSemaphoreGive(xBinarySemaphore3) != pdPASS)
                Serial.println("Erro ao enviar para semáforo 3");
            break;
        case A03: // Robô retorna para varredura - mov1Task
            if (xSemaphoreGive(xBinarySemaphore1) != pdPASS)
                Serial.println("Erro ao enviar para semáforo 1");
            break;  
        case A04: // Robô se movimenta até o fogo e emite alertas - apaga_fogoTask
            if (xSemaphoreGive(xBinarySemaphore4) != pdPASS)
                Serial.println("Erro ao enviar para semáforo 4");
            break;   
        case A05: // Robô desvia do foco de incêndio apagado  - desvioTask
            if (xSemaphoreGive(xBinarySemaphore3) != pdPASS)
                Serial.println("Erro ao enviar para semáforo 3");
            break;  
        case A06: // Robô é desativado - desligamentoTask
            if (xSemaphoreGive(xBinarySemaphore2) != pdPASS)
                Serial.println("Erro ao enviar para semáforo 1");
            break;         
    }

    return retval;
}

void iniciaMaquinaEstados() {
    for (int i = 0; i < NUM_ESTADOS; i++) {
        for (int j = 0; j < NUM_EVENTOS; j++) {
            acao_matrizTransicaoEstados[i][j] = NENHUMA_ACAO;
            proximo_estado_matrizTransicaoEstados[i][j] = i;
        }
    }
    proximo_estado_matrizTransicaoEstados[PARADO][ON] = VARREDURA;
    acao_matrizTransicaoEstados[PARADO][ON] = A01;

    proximo_estado_matrizTransicaoEstados[VARREDURA][OFF] = PARADO;
    acao_matrizTransicaoEstados[VARREDURA][OFF] = A06;

    proximo_estado_matrizTransicaoEstados[VARREDURA][OBSTACULO] = DESVIO;
    acao_matrizTransicaoEstados[VARREDURA][OBSTACULO] = A02;

    proximo_estado_matrizTransicaoEstados[OBSTACULO][OFF] = PARADO;
    acao_matrizTransicaoEstados[OBSTACULO][OFF] = A06;

    proximo_estado_matrizTransicaoEstados[DESVIO][SEM_OBSTACULO] = VARREDURA;
    acao_matrizTransicaoEstados[DESVIO][SEM_OBSTACULO] = A03;

    proximo_estado_matrizTransicaoEstados[DESVIO][OFF] = PARADO;
    acao_matrizTransicaoEstados[DESVIO][OFF]= A06;

    proximo_estado_matrizTransicaoEstados[VARREDURA][CHAMA] = BOMBEIRO;
    acao_matrizTransicaoEstados[VARREDURA][CHAMA]= A04;

    proximo_estado_matrizTransicaoEstados[BOMBEIRO][OFF] = PARADO;
    acao_matrizTransicaoEstados[BOMBEIRO][OFF]= A06;

    proximo_estado_matrizTransicaoEstados[BOMBEIRO][SEM_CHAMA] = DESVIO;
    acao_matrizTransicaoEstados[BOMBEIRO][SEM_CHAMA]= A05;

}

void obterEventoTask(void *param) {
    int lastButtonState = HIGH;
    int currentState = PARADO;
    int lastFlameState = 0;

    for (;;) {
        int codigoEvento = NENHUM_EVENTO;
        int buttonState = digitalRead(BUTTON_PIN);

        if (buttonState == LOW && lastButtonState == HIGH) { // Button pressed
            if (currentState == PARADO) {
                codigoEvento = ON;
                currentState = VARREDURA;
            } else {
                codigoEvento = OFF;
                currentState = PARADO;
            }
        }

        lastButtonState = buttonState;

        if (codigoEvento != NENHUM_EVENTO) {
            BaseType_t xStatus = xQueueSendToBack(xQueue, &codigoEvento, 0);
            if (xStatus != pdPASS) {
                Serial.println("Nao foi possivel escrever na fila.");
            }
        }

        if (currentState != PARADO && codigoEvento != OFF) {
            long startTime = millis();
            bool obstacleDetected = false;
            while (millis() - startTime < 500) {
                long distance = measureDistance();
                if (distance > DISTANCE_THRESHOLD) {
                    obstacleDetected = false;
                    break;
                }
                obstacleDetected = true;
                vTaskDelay(pdMS_TO_TICKS(100)); // Delay for a short period
            }

            if (detectaChama()){
              codigoEvento = CHAMA;
              lastFlameState = 1;
                BaseType_t xStatus = xQueueSendToBack(xQueue, &codigoEvento, 0);
                if (xStatus != pdPASS) {
                    Serial.println("Nao foi possivel escrever na fila.");
                }
              continue;
            }

            if (lastFlameState == 1){
                  codigoEvento = SEM_CHAMA;
                  lastFlameState = 0;
                  BaseType_t xStatus = xQueueSendToBack(xQueue, &codigoEvento, 0);
                  if (xStatus != pdPASS) {
                      Serial.println("Nao foi possivel escrever na fila.");
                  }
                  continue;
            }
            

            if (obstacleDetected) {
                codigoEvento = OBSTACULO;
                BaseType_t xStatus = xQueueSendToBack(xQueue, &codigoEvento, 0);
                if (xStatus != pdPASS) {
                    Serial.println("Nao foi possivel escrever na fila.");
                }
            }
            else{
              codigoEvento= SEM_OBSTACULO;
              BaseType_t xStatus = xQueueSendToBack(xQueue, &codigoEvento, 0);
              if (xStatus != pdPASS) {
                Serial.println("Nao foi possivel escrever na fila.");
              }
            }
        }

        delay(50); // Debounce delay
    }
}

// Sensor de chamas
int detectaChama(){
  int flame1_read = analogRead(MEIO_AO_PIN);
  int flame2_read = analogRead(ESQ_AO_PIN);
  int flame3_read = analogRead(DIR_AO_PIN);
  bool flame = false;
  if((flame1_read < 120) || (flame2_read < 120) || (flame3_read < 120)){
    flame = true;
    Serial.println("Tem fogo");
  }

  return flame;
}

// Sensor Ultrassônico
long measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH);
    long distance = duration * 0.034 / 2;

    return distance;
}

volatile int estadoAtual;

void processarEventoTask(void *param) {
    estadoAtual = PARADO;

    for (;;) {
        int codigoEvento;
        BaseType_t xStatus = xQueueReceive(xQueue, &codigoEvento, portMAX_DELAY);
        if (xStatus == pdPASS) {
            int proximoEstado = proximo_estado_matrizTransicaoEstados[estadoAtual][codigoEvento];
            int acao = acao_matrizTransicaoEstados[estadoAtual][codigoEvento];

            if (acao != NENHUMA_ACAO) {
                executarAcao(acao);
            }

            estadoAtual = proximoEstado;
        }
    }
}

void setup() {
    Serial.begin(9600);

    xBinarySemaphore1 = xSemaphoreCreateBinary();
    xBinarySemaphore2 = xSemaphoreCreateBinary();
    xBinarySemaphore3 = xSemaphoreCreateBinary();
    xBinarySemaphore4 = xSemaphoreCreateBinary();
    xQueue = xQueueCreate(10, sizeof(int));

    xTaskCreate(obterEventoTask, "Obter Evento Task", 4096, NULL, 1, NULL);
    xTaskCreate(processarEventoTask, "Processar Evento Task", 4096, NULL, 1, NULL);
    xTaskCreate(mov1Task, "Led Buzzer Task", 4096, NULL, 1, &mov1TaskH);
    xTaskCreate(desligamentoTask, "Led Buzzer Task2", 4096, NULL, 1, &desligamentoTaskH);
    xTaskCreate(apaga_fogoTask, "Led Buzzer Task3", 4096, NULL, 1, &apaga_fogoTaskH);
    xTaskCreate(desvioTask, "Desvio Task", 4096, NULL, 1, &desvioTaskH);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(MEIO_AO_PIN, INPUT);
    pinMode(ESQ_AO_PIN, INPUT);
    pinMode(DIR_AO_PIN, INPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(spd, OUTPUT);
    pinMode(BOMBA,OUTPUT);

    digitalWrite(BOMBA, HIGH);


    SerialBT.begin("Robo_Bombeiro");
    Serial.println("Agora é só parear...");
    iniciaMaquinaEstados();

    Serial.begin(115200);
    myservo.attach(SERVO_PIN);
}


void loop() {
    // Nothing here
}

// Robô se desloca para frente
void mov1Task(void *arg) {
    for (;;) {
        if (xSemaphoreTake(xBinarySemaphore1, portMAX_DELAY) == pdTRUE) {
            Serial.println("Mov1.");
            SerialBT.println("Robô Bombeiro acionado, procurando incêndio");
            while (estadoAtual == VARREDURA) {
                digitalWrite(in1, LOW);
                digitalWrite(in2, HIGH);
                digitalWrite(in3, HIGH);
                digitalWrite(in4, LOW);
                analogWrite(spd, 75); // speed up

                digitalWrite(LED_PIN, HIGH);
                vTaskDelay(pdMS_TO_TICKS(500));
                digitalWrite(LED_PIN, LOW);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
          digitalWrite(in4, LOW);
          analogWrite(spd, 0); // speed up
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Movimentação e ações desabilitadas
void desligamentoTask(void *arg) {
    for (;;) {
        if (xSemaphoreTake(xBinarySemaphore2, portMAX_DELAY) == pdTRUE) {
            Serial.println("Apitar.");
            SerialBT.println("Robô Bombeiro desligando");
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);
            analogWrite(spd, 0); // speed up
            while (estadoAtual == PARADO) {
                digitalWrite(BUZZER_PIN, HIGH);
                //vTaskDelay(pdMS_TO_TICKS(1000));
            }
            digitalWrite(BUZZER_PIN, LOW);
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Robô desvia de um obstáculo
void desvioTask(void *arg) {
    for (;;) {
        if (xSemaphoreTake(xBinarySemaphore3, portMAX_DELAY) == pdTRUE) {
            Serial.println("Obstáculo detectado");
            SerialBT.println("Desvio ativado");
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);
            vTaskDelay(pdMS_TO_TICKS(10));
            while (estadoAtual == DESVIO){
              digitalWrite(in1, LOW);
              digitalWrite(in2, HIGH);
              digitalWrite(in3, LOW);
              digitalWrite(in4, HIGH);
              analogWrite(spd, 50); // speed up; 
              vTaskDelay(pdMS_TO_TICKS(10));          
            }
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Robô apaga fogo e desvia do local
void apaga_fogoTask(void *arg) {
    for (;;) {
        if (xSemaphoreTake(xBinarySemaphore4, portMAX_DELAY) == pdTRUE) {
            Serial.println("Apitar.");
            SerialBT.println("FOGO!!");
            int flame1_read = analogRead(MEIO_AO_PIN);
            int flame2_read = analogRead(ESQ_AO_PIN);
            int flame3_read = analogRead(DIR_AO_PIN);
            int i = 0;
            if (flame2_read < flame1_read){
              while (i < 100){
                digitalWrite(in1, LOW);
                digitalWrite(in2, HIGH);
                digitalWrite(in3, LOW);
                digitalWrite(in4, HIGH);
                analogWrite(spd, 50); // speed up;
                i++; 
                vTaskDelay(pdMS_TO_TICKS(10)); 
              }
            }
            if (flame1_read > flame3_read){
              while (i < 100) {
                digitalWrite(in1, HIGH);
                digitalWrite(in2, LOW);
                digitalWrite(in3, HIGH);
                digitalWrite(in4, LOW);
                analogWrite(spd, 50); // speed up;
                i++;  
                vTaskDelay(pdMS_TO_TICKS(10)); 
              }
            }

            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);
            analogWrite(spd, 0); // speed up; 

            while (estadoAtual == BOMBEIRO) {
                digitalWrite(LED_PIN, HIGH);
                digitalWrite(BUZZER_PIN, HIGH);
                digitalWrite(BOMBA, LOW);
                  int angle1 = 70;  // Ângulo à direita
                  int angle2 = 0;   // Ângulo central
                  int angle3 = -70; // Ângulo à esquerda
                  
                  // Movimento do ponto inicial (20 graus) ao ponto superior do "U" (0 graus)
                  myservo.write(angle1);
                  delay(500); // Espera 1 segundo

                  // Movimento para o meio do "U" (0 graus)
                  myservo.write(angle2);
                  delay(500); // Espera 1 segundo

                  // Movimento para o outro lado do "U" (-20 graus)
                  myservo.write(angle3);
                  delay(500); // Espera 1 segundo

                  // Movimento de volta para o ponto médio (0 graus)
                  myservo.write(angle2);
                  delay(500); // Espera 1 segundo

                  // Movimento de volta ao ponto inicial (20 graus)
                  myservo.write(angle1);
                  delay(500); // Espera 1 segundo

                  myservo.write(angle2);
                  delay(500); // Espera 1 segundo
            }
            digitalWrite(BUZZER_PIN, LOW);
            digitalWrite(LED_PIN, LOW);
            digitalWrite(BOMBA, HIGH);
        }
    vTaskDelay(pdMS_TO_TICKS(10));
    }
}