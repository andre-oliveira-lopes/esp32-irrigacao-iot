#include "arduino_secrets.h"
#include <Wire.h>              // Nessesário para utilizar Display LCD
#include <LiquidCrystal_I2C.h> // Nessesário para utilizar Display LCD
#include <BluetoothSerial.h>   // Nessesário para utilizar Bluetooth da Esp
#include <DHTesp.h>            // Nessesário para utilizar sensor de Temperatura e Umidade DHT
#include <WiFi.h>              // Permite conectar o ESP32 a redes Wi-Fi
#include <HTTPClient.h>        // Facilita o envio de requisições HTTP (GET, POST, etc.)
#include <ArduinoJson.h>       // Utilizada para criar, analisar e manipular dados no formato JSON
#include "thingProperties.h"   // Arquivo gerado pela plataforma Arduino IoT Cloud para configurar propriedades e conexões do dispositivo

// ==============================  DECLARAÇÃO ANTECIPADA DA FUNÇÕES ==============================
void verificarConexaoWiFi();                                               // Gerencia a conexão Wi-Fi, tentando reconectar se necessário.
// ----------------------------------------------------
void lerSensorDHT();
  void enviarMediaDHT();
// ----------------------------------------------------
void lerSensorLDR(); 
  void enviarMediaLDR();                                                   // NOVA FUNÇÃO: envia a média a cada 60 segundos
  void analisarLuminosidade(int luz); 
  void executarAcao_solAparece();                                          // Nova função para ser usada futuramente
  void executarAcao_solDesapareceu();                                      // Nova função para ser usada futuramente
// ----------------------------------------------------
int lerMultiplexador();                                                    // Função que lê o valor analógico do canal atual do multiplexador
  void coletarLeituras();                                                  // Função que gerencia a leitura não bloqueante e acumula valores
  void calcularMedia60s();                                                 // Calcula a média das leituras acumuladas a cada 60 segundos
  void percentualLeituras();                                               // Esta função converte as leituras médias (valores brutos do sensor) em uma porcentagem
  void exibirLeituras();                                                   // Exibe as médias calculadas no serial e Bluetooth
  void imprimirMediaPinoEspecifico(byte pino);                             // Exibe a média de um canal específico
  void zerarTudoFree();                                                    // Zera os acumuladores e variáveis para nova coleta
// ----------------------------------------------------
void executarAcao_solAparece();                                            // Inicia a irrigação quando o sol aparece (ao amanhecer)
void executarAcao_solDesaparece();                                         // Inicia o esvaziamento do cano e prepara o sistema para irrigação noturna
  void removerAguaQuente();                                                // Esvazia o cano para remover água quente acumulada durante o dia
  void modoSonoEsp32();                                                    // Coloca a ESP32 em modo de sono profundo por 10 horas (economia de energia)
  void processarIrrigacao(bool entraEmSono, unsigned long agora);          // Gerencia a irrigação automática dos canais com lógica de tempo e tentativas
// ----------------------------------------------------
String seguirRedirecionamento(String url);                                 // Segue redirecionamentos de URLs em requisições HTTP.
  bool escreverEmLista(String identificacao, int numDados, float dados[]); // Envia uma lista de dados para uma planilha online.
  bool escreverEmCelula(String identificacao, String celula, String dado); // Escreve um dado específico em uma célula da planilha online.
  String lerCelula(String identificacao, String celula);                   // Lê o valor de uma célula específica da planilha online.
  String lerLinha(String identificacao, int linha);                        // Lê todos os dados de uma linha específica da planilha online.
  void montarCabecalho(String _boardID, const String& colunaInicial, const std::vector<String>& cabecalhos); // Garante que o cabeçalho da planilha esteja correto, escrevendo-o se necessário.

void lerComandosDaPlanilha();                            // Lê comandos de controle (LED, relés) da planilha online.
void atualizarValoresPlanilhaNuvem();                    // Atualiza estados de dispositivos e simula leituras de sensores para a planilha e nuvem.
  void atualizarEstado(int pino, String nome, bool estadoDesejado, float &estadoAtual); // Controla o estado (ligado/desligado) de um pino digital e atualiza seu valor.
void registrarDadosPlanilha();                           // Compila e envia todos os dados de sensores e atuadores para a planilha.

void onLedTesteChange();                                 // Executada quando o estado do LED é alterado via Arduino IoT Cloud.
  void onReleAChange();                                  // Executada quando o estado do Relé A é alterado via Arduino IoT Cloud.
  void onReleBChange();                                  // Executada quando o estado do Relé B é alterado via Arduino IoT Cloud.
  void onTemperaturaNuvemChange();                       // Executada quando a temperatura é atualizada via Arduino IoT Cloud.
  void onUmidadeNuvemChange();                           // Executada quando a umidade do ar é atualizada via Arduino IoT Cloud.
  void onLuminosidadeNuvemChange();                      // Executada quando a luminosidade é atualizada via Arduino IoT Cloud.
  void onUmidadeSoloNuvem3Change();                      // Executada quando a umidade do solo (canal 3) é atualizada via Arduino IoT Cloud.
  void onUmidadeSoloNuvem10Change();                     // Executada quando a umidade do solo (canal 10) é atualizada via Arduino IoT Cloud.
  void onUmidadeSoloNuvem14Change();                     // Executada quando a umidade do solo (canal 14) é atualizada via Arduino IoT Cloud.
// ----------------------------------------------------
void aguardar(unsigned long tempo_ms);                   // Função auxiliar para aguardar sem bloquear 
void atualizarIndicador(unsigned long tempoAtualMillis); // Pontinhos a cada segundo

// ====================== Configurações da rede Wi-Fi ======================
const char* ssid = "Pandora";                            // Nome da rede Wi-Fi à qual o ESP32 irá se conectar (SSID)
const char* password = "84759556";                       // Senha da rede Wi-Fi correspondente ao SSID acima

// ====================== URL do Web App do Google Apps Script ======================
const char* googleScriptURL = "https://script.google.com/macros/s/AKfycbx3ThbOom2Eo9b-YseFwPmGe6vPRr75rYGoaR_trRfnhnDPC5ekwWq3rHJJ5ZWZ9m0A/exec";

// ============ DEFINIÇÕES LCD ============
#define ENDERECO_LCD 0x27
#define COLUNAS_LCD 16
#define LINHAS_LCD 2
LiquidCrystal_I2C lcd(ENDERECO_LCD, COLUNAS_LCD, LINHAS_LCD);

// ============  DEFINIÇÕES DO SENSOR DHT11 ============ 
#define PINO_DHT11 13
DHTesp dht;

// ============  DEFINIÇÕES DO SENSOR LDR ============ 
#define PINO_LDR 15

// ==================== DEFINIÇÃO DOS PINOS DO MULTIPLEXADOR ====================
#define SIG 32 // Pino de entrada analógica que lê o sinal do multiplexador
#define S0 27  // Pino seletor 0 do multiplexador
#define S1 26  // Pino seletor 1 do multiplexador
#define S2 25  // Pino seletor 2 do multiplexador
#define S3 33  // Pino seletor 3 do multiplexador
// #define EN_PIN 0 // O pino EN (Enable) do CD74HC4067 deve estar em LOW (no GND) para o MULTIPLEXADOR funcionar.

// ============ DEFINIÇÕES DOS RELÉS ============
#define RELE_PINO_X 14                              // Pino conectado ao relé responsável por esvaziar o cano. 
#define RELE_PINO_A 19                              // Pino do relé para irrigar o pé de manga
#define RELE_PINO_B 3                               // Pino do relé para irrigar o pé de goiaba
#define LIMIAR_SECO 10                              // Umidade mínima aceitável antes de iniciar irrigação (%)

// Instanciando obejeto do tipo BluetoothSerial com nome de SerialBT
BluetoothSerial SerialBT;

// ============ Controle de Temporizador para leitura e alternância ============ 
// ============ WiFi ============ 
bool wifiConectado = false;                            // Estado da conexão Wi-Fi
bool tentandoConectar = false;                         // Controle para evitar WiFi.begin() repetido
unsigned long tempoUltimaVerificacaoWiFi = 0;
const unsigned long intervaloVerificacaoWiFi = 5000;   // 5 segundos

// ============ DHT11 ============
unsigned long ultimoTempoDHT = 0; // Recebe a ultima vez que houve a leitura do sensor DHT
const unsigned long intervaloLeituraDHT = 5000; // Tempo em milissegundos para fazer a proxima leitura do sensor (5 segundos)
const unsigned long intervaloMediaDHT = 60000; // Média a cada 60 segundos
unsigned long ultimoTempoMediaDHT = 0;  // Armazena o último tempo da média
float somaTemp = 0.0; // Variável numeradora para média 
float somaUmid = 0.0; // Variável numeradora para média
float mediaTemp = 0.0; 
float mediaUmid = 0.0; 
int contAmostrasValidasDHT = 0; // Variável denominadora para média 
// Variáveis globais para armazenar a última leitura válida
// float ultimaTemp = 0.0; // Use float para armazenar temperaturas com decimais
// float ultimaUmid = 0.0; // Use float para armazenar umidade com decimais

//  ============ LDR ============
unsigned long ultimoTempoLDR = 0; // Variáveis de Temporizadores: Recebe a ultima vez que houve a leitura do sensor LDR
const unsigned long intervaloLDR = 5000; // Variáveis de Temporizadores: Tempo em milissegundos para fazer a proxima leitura do sensor (5 segundos)
unsigned long ultimoTempoMediaLDR = 0; // Variáveis de Temporizadores
const unsigned long intervaloMediaLDR = 60000; // Variáveis de Temporizadores: Média a cada 60 segundos

float mediaLuz = 0; //  Variáveis de controle
unsigned long tempoInicioBaixaLuz = 0; //  Variáveis de controle
unsigned long inicioLuzAmanhecendo = 0; //  Variáveis de controle
bool acaoExecutadaAoAnoitecer = false; //  Variáveis de controle
bool acaoExecutadaAoAmanhecer = false; //  Variáveis de controle
bool estaDeNoite = false; //  Variáveis de controle
long somaLuz = 0; // Variáveis para média
int contAmostrasValidasLDR = 0; // Variáveis para média

//  ============ MULTIPLEXADOR ============
unsigned long tempoUltimaLeituraCanal = 0; // // Guarda o Tempo que a última leitura de canal ocorreu
const unsigned long intervaloLeituraPorCanal = 20; // Tempo (ms) entre leituras de cada canal para estabilização do sinal (ex: a cada 5ms lê um canal). 
unsigned long tempoUltimaExibicao = 0; // Guarda o tempo da última exibição das médias 
const unsigned long intervaloExibicao = 60000; // Intervalo para calcular e exibir a média (60 segundos)
// Variáveis globais para exibição no LCD
unsigned long ultimaTrocaLCD = 0;      // Marca o tempo da última atualização
const unsigned long intervaloLCD = 2000; // Tempo entre trocas (em milissegundos)
int indiceLCD = 0;                     // Índice do pino atual sendo exibido

const byte numeroPino = 16;
int sensoresMinimos[numeroPino] = {                         // Inicializa sensoresMinimos com 4095 (valor mais alto possível para leitura analógica)
  1720,  // Pino 0
  1750,  // Pino 1
  1745,  // Pino 2
  2200,  // Pino 3 <------------------- estou usando 
  1730,  // Pino 4
  1755,  // Pino 5
  1500,  // Pino 6 
  1760,  // Pino 7
  1740,  // Pino 8
  1735,  // Pino 9
  1250,  // Pino 10 <------------------- estou usando. 1300 - 96%
  1720,  // Pino 11
  1745,  // Pino 12
  1765,  // Pino 13
  2300,  // Pino 14 <------------------- estou usando 
  1740   // Pino 15
};
int sensoresMaximos[numeroPino] = {                          // Inicializa sensoresMaximos com 0 (valor mais baixo possível)
  3200,  // Pino 0
  3220,  // Pino 1
  3210,  // Pino 2
  5000,  // Pino 3 <------------------- estou usando 
  3230,  // Pino 4
  3240,  // Pino 5
  3100,  // Pino 6 
  3225,  // Pino 7
  3215,  // Pino 8
  3200,  // Pino 9 
  4095,  // Pino 10 <------------------- estou usando 
  3195,  // Pino 11
  3220,  // Pino 12
  3245,  // Pino 13
  4095,  // Pino 14 <------------------- estou usando 
  3225   // Pino 15
};

int canalAtual = 0; // Canal atual do multiplexador que está sendo lido (de 0 a 15)
int valores_analogicos[16]; // Armazena a última leitura instantânea de cada canal. Essa aqui eu vou deixar pra quando eu quiser saber o valor bruto que o sensor detectou.
long somaLeituras[16] = {0}; // // Acumula a soma das leituras de cada canal para cálculo da média
int medias[16] = {0}; // Armazena a média calculada para cada canal
int mediasPercentual[numeroPino];  // Array para armazenar os valores de umidade em percentual de 0 a 100
int totalAmostrasJanela = 0; // Quantidade total de leituras feitas durante a janela de tempo (60s)

// Pinos e tabela do multiplexador
const int seletores[4] = {S3, S2, S1, S0};    // Seletores do multiplexador em vetor para facilitar o controle
const byte tabelaMUX[16][4] = {               // Tabela binária para acessar os canais do multiplexador. 
  {0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1}, // 0 em decimal - 1 em decimal - 2 em decimal - 3 em decimal 
  {0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1}, // 4 em decimal - 5 em decimal - 6 em decimal - 7 em decimal
  {1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1}, // 8 em decimal - 9 em decimal - 10 em decimal - 11 em decimal
  {1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1}  // 12 em decimal - 13 em decimal - 14 em decimal - 15 em decimal
};

// ============ Indicador Visual ============ 
unsigned long ultimoTempoIndicador = 0;
const unsigned long intervaloIndicador = 1000; // Tempo em milissegundos para fazer aparecer um (.) e ir acrescentando até 5 pontos (1 segundo)
int contadorPontos = 0;

// ============ Relé ============ 
const unsigned long tempoIrrigando = 60000;         // Tempo padrão de irrigação (60 segundos)
const unsigned long tempoEntreRele = 5000;          // Tempo de espera entre desligar um relé e ligar o próximo (5 segundos)
const unsigned long tempoMaximoIrrigacao = 120000;  // Tempo máximo de irrigação por segurança (120 segundos)

const int umidadeDesejada = 85;                     // Umidade alvo após irrigação
const int maxTentativas = 4;                        // Número máximo de tentativas de irrigar o mesmo canal
bool irrigando = false;                             // Indica se o sistema está irrigando neste momento
bool aguardandoProximo = false;                     // Indica se está esperando para ligar o próximo relé
bool ultrapassouLimiteSeguranca = false;            // Indica se o tempo máximo de irrigação foi ultrapassado
unsigned long tempoInicioIrrigacao = 0;             // Marca o momento em que iniciou a irrigação
unsigned long tempoInicioEspera = 0;                // Marca o momento em que começou a aguardar entre revezamento de relés
int indiceReleAtual = 0;                            // Índice atual no array de relés
int tentativasAtual = 0;                            // Número de tentativas já feitas para o canal atual

const int reles[16] = {                             // Array de pinos dos relés (somente os válidos serão usados)
  RELE_PINO_A, RELE_PINO_B, -1, -1, -1, -1, 
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
};

const char* nomesReles[16] = {                      // Array de ponteiros para strings constantes. Mapeia os índices dos relés aos seus respectivos nomes textuais. 
  "RELE_PINO_A", "RELE_PINO_B", "N/A", "N/A",       // Cada elemento armazena o endereço de memória de uma string literal. 
  "N/A", "N/A", "N/A", "N/A", "N/A", "N/A", 
  "N/A", "N/A", "N/A", "N/A", "N/A", "N/A"
};

bool esvaziamentoFeito = false;                      // Indica se o esvaziamento do cano já foi concluído. essa operação ocorra somente uma vez por noite.
bool esvaziandoCano = false;                         // Indica se o sistema está atualmente esvaziando o cano
unsigned long tempoRemoverAgua = 0;                  // Armazena o tempo (em millis) em que o esvaziamento começou. Usado para controlar quando ele deve terminar.
const unsigned long tempoEsvaziamento = 120000;      // Tempo total de esvaziamento do cano (em milissegundos). 2 minutos

bool preparandoSono = false;                         // Sinaliza que a preparação para entrar em sono profundo já começou.
unsigned long tempoInicioPreSono = 0;                // Armazena o "momento atual" (em milissegundos desde que o ESP32 foi ligado).

// ============ Nuvem ============ 
// Variáveis de sensores e atuadores
// Elas tiveram que ser declaradas com o tipo froat pois a função que passava elas como parâmetro pedia assim, para que o compilador não faça uma conversão, eu já fiz.
// Aproveitei essa declaração para usar elas para passar o seu valor atual para a nuvem e a planilha. 
float estadoLed = 0.0;
float temperatura = 0.0;
float umidadeAr = 0.0;
float luminosidade = 0.0;
float umidadeSolo3 = 0.0;
float umidadeSolo10 = 0.0;
float umidadeSolo14 = 0.0;

// Pino
const int ledpin     = 2;                             // Pino do LED Azul

// Comandos recebidos da planilha e estados
String comandoLed   = "";                             // Guarda o comando lido da planilha
String comandoReleA = "";                             // Guarda o comando lido da planilha
String comandoReleB = "";                             // Guarda o comando lido da planilha

// Estados atuais (para enviar para planilha)
float estadoLedAtual   = 0.0;                         // Cria uma variável que representará esse estado como número (1.0 para ligado, 0.0 para desligado)
float estadoReleA      = 0.0;                         // Armazena o estado atual do Relé A (0.0 = desligado, 1.0 = ligado)
float estadoReleB      = 0.0;                         // Armazena o estado atual do Relé B (0.0 = desligado, 1.0 = ligado)

bool estadoDesejadoLed   = false;                     // Estado desejado do LED (true = ligar, false = desligar)
bool estadoDesejadoReleA = false;                     // Estado desejado do Relé A
bool estadoDesejadoReleB = false;                     // Estado desejado do Relé B

// Variáveis de controle
unsigned long tempoUltimaLeitura = 0;                  // Marca do tempo da última leitura
const unsigned long intervaloLeitura = 60000;          // Tempo entre leituras (ms)

// ==============================
//             SETUP
// ==============================

void setup() {
  Serial.begin(115200);   // Inicializa comunicação serial (Esp32 <---> PC)
  SerialBT.begin("ESP32_IRRIGACAO"); // Nome do Dispositivo Bluetooth que eu vou me conectar para visualizar as informações (Esp32 <---> Celular)
  // Caso a Esp32 apresente erro ao se conectar com o Bluetooth do celular reinicie ela manialmente apertando no botão "EN", lago oposto ao botão "BOOT".
  lcd.init(); // Inicializa o display LCD (Esp32 <---> Display)
  
  // Alternativa ao delay() usando millis(). Aguarda 1 segundos sem bloquear o restante do loop futuramente
  // é ideal para configurações iniciais, não impedir que o Esp32 responda a eventos e outros códigos pois o setup só executa uma vez.
  aguardar(2000); // Aguarda 2 segundo para inicialização. é ideal para configurações iniciais, não impedir que o Esp32 responda a eventos e outros códigos pois o setup só executa uma vez. 
  lcd.backlight(); // Acende a tela
  lcd.clear(); // limpa o Display
  lcd.setCursor(0, 0); // coloca o cursor na primeira coluna e linha
  
  // MOSTRAR DADOS: Sistema iniciado - Mostrando nas 3 saídas
  lcd.print("Sistema Iniciado"); // Display LCD (Máximo de caracteres: 16)
  Serial.println("Inicializada a comunicação entre Esp32 e PC"); // Monitor Serial da IDE arduino
  SerialBT.println("Sistema iniciado via Bluetooth"); // Tela do celular, Aplicativo da PlayStore: Serial Bluetooth Terminal
  
  // a condição: 1001 < 1000 é FALSO, então sai do while. A variável tempoInicio não muda, ela guarda o valor fixo no momento da "partida" (como dar start no cronômetro). 
  // Já millis() continua subindo automaticamente.
  aguardar(1000); // Aguarda mais 1 segundo para exibição clara das mensagens. Só use a função Delay() no função principal do código. Nunca use no Loop! 
  lcd.clear(); // limpa o Display

  // Configura os pinos
  pinMode(ledpin,OUTPUT);
  pinMode(RELE_PINO_X, OUTPUT); // utilizado para esvaziar cano
  // OBS importante: Não é obrigatório configurar pinos para entrada analógica (DHT, LDR), mas bom para clareza.  
  pinMode(RELE_PINO_A, OUTPUT); // Relé: o ESP32 pode enviar sinais elétricos para controlar o relé conectado.
  pinMode(RELE_PINO_B, OUTPUT); // Relé: o ESP32 pode enviar sinais elétricos para controlar o relé conectado. 
  dht.setup(PINO_DHT11, DHTesp::DHT11); // Inicializa o sensor DHT especificando o pino e o tipo
  pinMode(PINO_LDR, INPUT); 

  // pinMode(EN_PIN, OUTPUT); // Não precisa de configuração por software se ligado ao GND no hardware.
  pinMode(SIG, INPUT); // Configura pino SIG como entrada analógica para ler os sinais do multiplexador
  // Define os pinos do multiplexador como saída para controlar os canais do multiplexador
  pinMode(S0,OUTPUT);
  pinMode(S1,OUTPUT);
  pinMode(S2,OUTPUT);
  pinMode(S3,OUTPUT);
  // digitalWrite(EN_PIN, LOW); // Não precisa de configuração por software se ligado ao GND no hardware.

  // função do arduino Cloud
  initProperties();                                     // Defined in thingProperties.h
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);    // Connect to Arduino IoT Cloud

  // Estado inicial: rele usado para esvaziar cano
  digitalWrite(RELE_PINO_X, LOW);
  // Estado inicial: Solenoide A e B desligado
  digitalWrite(RELE_PINO_A, LOW);
  digitalWrite(RELE_PINO_B, LOW);

  // MOSTRAR DADOS: No Display LCD, Monitor Serial da IDE arduino e Tela do celular.
  lcd.setCursor(0, 0); // coluna e linha
  lcd.print("A:OFF  B:OFF"); // (Máximo de caracteres: 16)
  Serial.println("Solenoide A desligado | Solenoide B desligado");
  SerialBT.println("Solenoide A desligado | Solenoide B desligado");

  // função do Wifi
  WiFi.begin(ssid, password);                    // Inicia tentativa de conexão Wi-Fi
  Serial.println("Iniciando conexão Wi-Fi...");  // (opcional) mostra status inicial

  // função do arduino Cloud
  setDebugMessageLevel(2);                       // Define o nível de mensagens de depuração (debug) para 2 (nível médio de detalhes)
  ArduinoCloud.printDebugInfo();                 // Imprime informações de depuração sobre a conexão com a Arduino Cloud

  // é recomendavel deixar esses 3 segundos para as informações ficarem organizadas ao apresentar para o usuario.
  //Aguarda 1 segundos sem bloquear o restante do loop futuramente
  aguardar(2000); // Aguarda mais 2 segundos para deixar as informações organizadas ao apresentar para o usuario.  
}

// ==============================
//        LOOP PRINCIPAL
// ==============================
void loop() {
  ArduinoCloud.update();
  unsigned long tempoAtualMillis = millis(); // Tempo desde que o ESP32 começou o loop. Obtém o tempo atual

    // Verifica Wi-Fi a cada 5 segundos
    if (tempoAtualMillis - tempoUltimaVerificacaoWiFi >= intervaloVerificacaoWiFi) {
      tempoUltimaVerificacaoWiFi = tempoAtualMillis;
      verificarConexaoWiFi(); // Tenta conectar ou se não der certo, tenta reconectar a cada 5s
  }

    if (!wifiConectado) return;  // Impede o resto da lógica sem Wi-Fi

      // Leitura do sensor DHT a cada 5 segundos
      if (tempoAtualMillis - ultimoTempoDHT >= intervaloLeituraDHT) { // Aguarda o intervalo antes da próxima leitura. É recomendável pelo menos 1 segundo para o sensor DHT11.
        ultimoTempoDHT = tempoAtualMillis;  // ultimoTempoDHT recebe a ultima vez que houve a Leitura do sensor DHT
        lerSensorDHT(); // depois do tempo programado, a função lerSensorDHT() é executada.
    }
        // Calcular e enviar a média a cada 60 segundos
        if (tempoAtualMillis - ultimoTempoMediaDHT >= intervaloMediaDHT) {
          ultimoTempoMediaDHT = tempoAtualMillis;
          enviarMediaDHT(); // Envia as médias calculadas e reseta as somas
      }
          // Leitura do sensor LDR a cada 5 segundos
          if (tempoAtualMillis - ultimoTempoLDR >= intervaloLDR) {
            ultimoTempoLDR = tempoAtualMillis;
            lerSensorLDR();
        }
            if (tempoAtualMillis - ultimoTempoMediaLDR >= intervaloMediaLDR) {
              ultimoTempoMediaLDR = tempoAtualMillis; 
              enviarMediaLDR(); // Enviar média a cada 60 segundos
              analisarLuminosidade(mediaLuz);
          }
                // --- Tarefa 1: Coletar e Acumular leituras (Não bloqueante, gerenciado por tempo) --- Verifica se já passou o tempo definido (intervaloLeituraPorCanal) desde a última leitura de *qualquer* canal. Para ver se é hora de ler o PRÓXIMO canal na sequência
                if (tempoAtualMillis - tempoUltimaLeituraCanal >= intervaloLeituraPorCanal) { // Este é o timer que controla a velocidade com que os canais são ciclados.
                  tempoUltimaLeituraCanal = tempoAtualMillis; // Atualiza o tempo da última leitura de canal. 
                  coletarLeituras(); // Chama a função que lê o canalAtual, acumula o valor e avança para o próximo canal.
              }
                    // --- Tarefa 2: Calcular média e exibir resultados a cada 60 segundos e resetar (agendada a cada intervaloExibicao) ---
                    if (tempoAtualMillis - tempoUltimaExibicao >= intervaloExibicao) { // Verifica se já passou o tempo definido para calcular e exibir as médias.
                      tempoUltimaExibicao = tempoAtualMillis; // Reseta o timer da exibição para o próximo ciclo. Atualiza o tempo da última exibição. 
                      
                      // Chama as funções
                      calcularMedia60s(); // Calcula as médias usando os dados acumulados na última janela de 60 segundos. 
                      percentualLeituras();  // Esta função converte as leituras médias (valores brutos do sensor) em uma porcentagem
                        // Aqui eu vou explicar qual dessas duas funções acima você deve usar: exibirLeituras(); ou imprimirMediaPinoEspecifico(byte pino).
                        // Se for usar exibirLeituras();: Significa que você comprou os 16 sensores de umidade do solo e está usando todos eles no multiplexador.
                        // Se for usar imprimirMediaPinoEspecifico(byte pino): Significa que você comprou menos que isso, e só vai precisar mostrar alguns pinos.
                        // o número que você passar como argumento (parametro) dessa função, vai ser o pino que será apresentado para o usuário.
                      // exibirLeituras(); // Exibe as médias calculadas (agora as médias dos últimos 60s) no monitor serial e via Bluetooth
                      imprimirMediaPinoEspecifico(3);  // Exibe média do canal 3
                      imprimirMediaPinoEspecifico(10); // Exibe média do canal 10
                      imprimirMediaPinoEspecifico(14); // Exibe média do canal 14
                  }
                        // Insere os dados da planilha e na nuvem
                        if (tempoAtualMillis - tempoUltimaLeitura >= intervaloLeitura) {
                          tempoUltimaLeitura = tempoAtualMillis;
                          lerComandosDaPlanilha();          // Lê os comandos da planilha se houver (por exemplo, liga/desliga LED e relés)
                          atualizarValoresPlanilhaNuvem();  // Com base na planilha atualiza os estados dos dispositivos e sensores ou com base nos sensores atualiza a planilha e o arduino cloud
                          registrarDadosPlanilha();         // Envia os dados coletados (estados e sensores) para a planilha
                      }

  zerarTudoFree();  // Limpa os acumuladores para começar a coletar dados para o próximo ciclo de média
  // indicador de atividade ou "animação de espera", mostrando um progresso a cada segundo.    
  atualizarIndicador(tempoAtualMillis); //Adicionei esse argumento na chamada dessa função
  // fim do loop. Volta a repetir tudo!
}

// ======================
// Tenta conectar ao Wi-Fi se ainda não conectado
// ======================

void verificarConexaoWiFi() {

  if (!wifiConectado) {                   // Verifica se o dispositivo ainda não está conectado ao Wi-Fi
    if (WiFi.status() == WL_CONNECTED) {  // Verifica se a conexão Wi-Fi foi estabelecida
      wifiConectado = true;
      tentandoConectar = false;
      Serial.println("Wi-Fi conectado com sucesso!");

      // Monta cabeçalho da planilha após conexão - Envia para a planilha os títulos das colunas que serão utilizados para registrar os dados
      montarCabecalho("dados", "A", { "Data completa", "Data", "Hora", "Estado do LED", "Estado Rele A", "Estado Rele B", "Temperatura (°C)", "Umidade (%)", "Luminosidade (%)", "Umidade Solo 3", "Umidade Solo 10", "Umidade Solo 14" });
    } else {
        Serial.println("Tentando reconectar ao Wi-Fi...");
        WiFi.disconnect();           // IMPORTANTE: desconecta antes de tentar novamente
        WiFi.begin(ssid, password);  // Tenta reconectar de tempos em tempos
      }
    }
  }

// ==============================
// FUNÇÃO: Leitura do sensor DHT11 (armazenando amostras válidas)
// ==============================

void lerSensorDHT() {
  // Função da biblioteca para leitura dos dados da constante PINO_DHT11 que vem do pino 18
  // float temp = dht.getTemperature(); // Usa dht.getTemperature() para obter a temperatura
  // float umid = dht.getHumidity();    // Usa dht.getHumidity() para obter a umidade
  TempAndHumidity th = dht.getTempAndHumidity(); // Outra forma de ler ambos os valores de uma vez

  // A biblioteca DHTesp retorna NaN (Not a Number) em caso de erro de leitura.
  if (!isnan(th.temperature) && !isnan(th.humidity) && 
        th.temperature >= 10 && th.temperature <= 60 &&
          th.humidity >= 10 && th.humidity <= 100) {   // Verifica se a leitura foi bem-sucedida e os valores estão dentro do intervalo confiável
    // Processar leitura
    somaTemp += th.temperature;
    somaUmid += th.humidity;
    contAmostrasValidasDHT++;
    
    // ultimaTemp = th.temperature; // Só se eu quiser ver os dados errados
    // ultimaUmid = th.humidity; // Só se eu quiser ver os dados errados
  }
}

// ==============================
// FUNÇÃO: Enviar médias calculadas
// ==============================

void enviarMediaDHT() {

  String dados;

    if (contAmostrasValidasDHT > 0) {
      mediaTemp = somaTemp / contAmostrasValidasDHT;
      mediaUmid = somaUmid / contAmostrasValidasDHT;

      dados = "Temp: " + String(mediaTemp, 1) + "°C | Umid: " + String(mediaUmid, 1) + "%"; // Mostra 1 casa decimal

      // MOSTRAR DADOS: Display LCD
      lcd.clear();
      lcd.setCursor(0, 0); // coluna e linha: primeira Coluna e primeira linha.  
      lcd.print("T:"); // " T:30°C U:55% "
      lcd.print(mediaTemp);
      lcd.print((char)223); // símbolo de grau
      lcd.print("C ");
      lcd.print("U:");
      lcd.print(mediaUmid);
      lcd.print("% ");

      // MOSTRAR DADOS: Monitor Serial da IDE arduino e Tela do celular (Envia os dados atuais)
      Serial.println(dados);
      SerialBT.println(dados);
  } /* else {   // Tratar erro

    // Leitura inválida ou falha — usa os últimos dados válidos, marcando com "¬"
    dados = "¬ Temp: " + String(ultimaTemp, 1) + "°C | ¬ Umid: " + String(ultimaUmid, 1) + "%"; // Mostra 1 casa decimal

    Serial.println("Erro na leitura do DHT!");
    SerialBT.println("Erro na leitura do DHT!");

    // MOSTRAR DADOS: Monitor Serial da IDE arduino e Tela do celular (Envia os dados anteriores)
    Serial.println(dados);
    SerialBT.println(dados);
  } */

    // Reseta acumuladores (Numerador e Denominador da média)
    somaTemp = 0.0;
    somaUmid = 0.0;
    contAmostrasValidasDHT = 0;
}

// ==============================
// FUNÇÃO: Leitura do sensor LDR
// ==============================

void lerSensorLDR() {

  // Lê valor analógico de 0 a 4095 (ESP32)
  int valorLDR = analogRead(PINO_LDR); 
  int percentualLuminosidadeLocal = map(valorLDR, 0, 4095, 100, 0); // 100% é claro, 0% escuro. Convertendo para porcentagem de luminosidade (opcional, inverso)

  // Acumula valores para média
  somaLuz += percentualLuminosidadeLocal; // <<<<< Atualiza o valor aqui
  contAmostrasValidasLDR++; // <<<<< Atualiza o valor aqui
}

// ==============================
// FUNÇÃO: ENVIA A MÉDIA DO LDR
// ==============================

void enviarMediaLDR() {

  if (contAmostrasValidasLDR > 0) {
    mediaLuz = somaLuz / (float)contAmostrasValidasLDR; // <<<<< Atualiza o valor da variavel global aqui
  } else {
    Serial.println("Sem leituras validas no ultimo minuto.");
    SerialBT.println("Sem leituras validas no ultimo minuto.");
  }
  // Reseta acumuladores
  somaLuz = 0;
  contAmostrasValidasLDR = 0;
}

// =====================================
// FUNÇÃO: ANALISA A LUMINOSIDADE COM BASE NA luz 
// =====================================

void analisarLuminosidade(float luz) {
  unsigned long agora = millis();
  String dados = "Luminosidade: " + String(luz) + "%";

  if (luz < 10) { // ANOITECER: Luz abaixo de 20%
    // MOSTRAR DADOS: Monitor Serial da IDE arduino e Tela do celular.
    SerialBT.println("=========== Leitura LDR ===========");
    SerialBT.print(dados);
    SerialBT.println(F(" - Nivel de luz baixo."));
    Serial.println("=========== Leitura LDR ===========");
    Serial.println(dados);
    Serial.println(F(" - Nivel de luz baixo."));
    // Mostrando no LCD 
    
    lcd.setCursor(0, 1); //coluna e linha: primeira Coluna e segunda linha.  
    lcd.print("Luz: ");
    lcd.print(luz);
    lcd.print("%     "); // espaço para apagar caracteres anteriores da leitura passada de luminosidade. 
    
      if (!estaDeNoite) {
        if (tempoInicioBaixaLuz == 0) {
          tempoInicioBaixaLuz = agora; // Começa a contar o tempo com baixa luminosidade
        }

        bool passou10Min = (agora - tempoInicioBaixaLuz >= 600000); // 10 minutos em milissegundos
        if (acaoExecutadaAoAnoitecer == false && passou10Min) {
        executarAcao_solDesaparece(); // pode ser a irrigação - ação ao anoitecer
        acaoExecutadaAoAnoitecer = true;
        estaDeNoite = true;
        acaoExecutadaAoAmanhecer = false;
        tempoInicioBaixaLuz = 0;
      }
    }
    inicioLuzAmanhecendo = 0; // Reset para amanhecer
  } else if (luz > 10 && luz <= 80){ // Luminosidade crescente (possível amanhecer)
      // MOSTRAR DADOS: Monitor Serial da IDE arduino e Tela do celular.
      SerialBT.println("=========== Leitura LDR ===========");
      SerialBT.print(dados);
      SerialBT.println(F(" - Nivel de luz normal."));
      Serial.println("=========== Leitura LDR ===========");
      Serial.println(dados);
      Serial.println(F(" - Nivel de luz normal."));
      // Mostrando no LCD 
      
      lcd.setCursor(0, 1); //coluna e linha: primeira Coluna e segunda linha.  
      lcd.print("Luz: ");
      lcd.print(luz);
      lcd.print("%     "); // espaço para apagar caracteres anteriores da leitura passada de luminosidade. 
      
      if (estaDeNoite) { // Só entra aqui após anoitecer.
        if (inicioLuzAmanhecendo == 0) {
        inicioLuzAmanhecendo = agora;
      }

      bool passou5Min = (agora - inicioLuzAmanhecendo >= 300000); // 5 minutos em milissegundos
      if (!acaoExecutadaAoAmanhecer && passou5Min) {
        executarAcao_solAparece(); // ação ao amanhecer
        acaoExecutadaAoAmanhecer = true;
        estaDeNoite = false;
        acaoExecutadaAoAnoitecer = false;
        inicioLuzAmanhecendo = 0; // Se a luz subiu, reseta os controles
      }
    }
  }
    else if (luz > 80) { // Luminosidade bem alta
    // MOSTRAR DADOS: Monitor Serial da IDE arduino e Tela do celular.
    SerialBT.println("=========== Leitura LDR ===========");
    SerialBT.print(dados);
    SerialBT.println(F(" - Nivel de luz alto."));
    Serial.println("=========== Leitura LDR ===========");
    Serial.println(dados);
    Serial.println(F(" - Nivel de luz alto."));
    // Mostrando no LCD 
    
    lcd.setCursor(0, 1); //coluna e linha: primeira Coluna e segunda linha.  
    lcd.print("Luz: ");
    lcd.print(luz);
    lcd.print("%     "); // espaço para apagar caracteres anteriores da leitura passada de luminosidade. 
    
    // Se a luz voltou a ficar forte (>80%), cancelamos qualquer contagem anterior, pois não estamos mais em transição de amanhecer ou anoitecer
    tempoInicioBaixaLuz = 0; // Cancela a contagem para início da irrigação ao anoitecer
    inicioLuzAmanhecendo = 0; // Cancela a contagem para início da irrigação ao amanhecer
    } else {
        Serial.println("Erro no Sensor!");
        SerialBT.println("Erro no Sensor!");
    }
 
  Serial.println();
  SerialBT.println();
} 

// ==============================
// FUNÇÃO: AÇÃO AO AMANHECER
// ==============================

void executarAcao_solAparece() {
  static bool acaoSolAapareceIniciada = false;      // Flag estática: garante que a inicialização ocorra uma vez.
  unsigned long agora = millis();                   // `millis()`: captura o tempo atual para controle não bloqueante.

  if (!acaoSolAapareceIniciada) {                   // Condição: verifica se a ação de "amanhecer" já foi executada.
    Serial.println("Executando ação: Amanheceu");   // Saída serial: informa que o amanhecer foi detectado.
    SerialBT.println("Executando ação: Amanheceu"); // Saída Bluetooth: informa que o amanhecer foi detectado.

    lcd.clear(); // limpa o Display
    lcd.setCursor(0, 0); // coluna e linha
    lcd.print("Executando..."); // (Máximo de caracteres: 16)
    lcd.setCursor(0, 1);
    lcd.print("Amanheceu..."); 

    acaoSolAapareceIniciada = true;                 // Atualiza flag: indica que a inicialização já ocorreu.

    irrigando = false;                              // Reseta estado: nenhuma irrigação ativa no início do dia.
    aguardandoProximo = false;                      // Reseta estado: sistema não espera entre relés.
    indiceReleAtual = 0;                            // Reseta controle: começa o ciclo de irrigação pelo primeiro relé.
    tentativasAtual = 0;                            // Reseta contador: cada canal tem tentativas disponíveis.
  }
  
  processarIrrigacao(false, agora);                 // Chamada de função: inicia/continua a lógica de irrigação diurna (sem entrar em deep sleep).
}

// ==============================
// FUNÇÃO: AÇÃO AO ANOITECER
// ==============================

void executarAcao_solDesaparece() {
  static bool acaoSolDesapareceIniciada = false;    // Flag estática: garante inicialização única ao "anoitecer".
  unsigned long agora = millis();                   // `millis()`: captura o tempo atual para controle não bloqueante.

  removerAguaQuente();                              // Chamada de função: inicia ou continua o processo de esvaziamento do cano.
  if (!esvaziamentoFeito) return;                   // Condição: aguarda o esvaziamento do cano ser concluído antes de prosseguir.

  if (!acaoSolDesapareceIniciada) {                 // Condição: verifica se a ação de "anoitecer" já foi executada.
    Serial.println("Executando ação: Anoiteceu");   // Saída serial: informa que o anoitecer foi detectado.
    SerialBT.println("Executando ação: Anoiteceu"); // Saída Bluetooth: informa que o anoitecer foi detectado.

    lcd.clear();                                    // limpa o Display
    lcd.setCursor(0, 0);                            // coluna e linha
    lcd.print("Executando...");                     // (Máximo de caracteres: 16)
    lcd.setCursor(0, 1);                            //  
    lcd.print("Anoiteceu...");                      //

    acaoSolDesapareceIniciada = true;               // Atualiza flag: indica que a inicialização já ocorreu.

    irrigando = false;                              // Reseta estado: nenhuma irrigação ativa no início da noite.
    aguardandoProximo = false;                      // Reseta estado: sistema não espera entre relés.
    indiceReleAtual = 0;                            // Reseta controle: começa o ciclo de irrigação (se houver) pelo primeiro relé.
    tentativasAtual = 0;                            // Reseta contador: cada canal tem tentativas disponíveis.
    // essas duas são um diferencial da executarAcao_solDesaparece(), pois tem envouvimento com removerAguaQuente().
    esvaziamentoFeito = false;                      // Reseta flag: permite que o esvaziamento ocorra novamente na próxima noite.
    esvaziandoCano = false;                         // Reseta estado: indica que o cano não está mais sendo esvaziado.
  }
  
  processarIrrigacao(true, agora);                  // Chamada de função: inicia/continua a lógica de irrigação noturna (preparando para deep sleep).
}

// ==============================
// FUNÇÃO: PROCESSO DE IRRIGAR PLANTAS
// ==============================

void processarIrrigacao(bool entraEmSono, unsigned long agora) {                      // Parâmetros: 'entraEmSono' (se deve ir para deep sleep) e 'agora' (tempo atual).

  // Início da lógica de finalização do ciclo de irrigação:
  if (indiceReleAtual >= numeroPino) {                                                // Condição: verifica se todos os canais de irrigação válidos foram processados.
    if (entraEmSono) {                                                                // Condição: se a função foi chamada com 'true' para deep sleep (modo noturno).
      modoSonoEsp32();                                                                // Chamada de função: coloca o ESP32 em sono profundo para economizar energia.
    } else {                                                                          // Condição: se a função foi chamada com 'false' (modo diurno).
      Serial.println("Irrigação finalizada (dia). Aguardando próximo ciclo...");      // Saída serial: informa o fim do ciclo diurno.
      SerialBT.println("Irrigação finalizada (dia). Aguardando próximo ciclo...");    // Saída Bluetooth: informa o fim do ciclo diurno.

      lcd.clear();                                                                    // limpa o Display
      lcd.setCursor(0, 0);                                                            // coluna e linha
      lcd.print("FIM IRRIGACAO");                                                     // (Máximo de caracteres: 16)
      lcd.setCursor(0, 1);                                                            //
      lcd.print("DIA");                                                               //
    }
    return;                                                                           // Encerra a execução da função: não há mais relés para processar neste ciclo.
  }

  int pinoRele = reles[indiceReleAtual];         	                                    // Variável local: obtém o número do pino do relé atual.
  int umidade = mediasPercentual[indiceReleAtual];                                    // Variável local: obtém o valor da umidade do solo para o canal atual.

  // Lógica para pular canais não utilizados:
  if (pinoRele == -1) {                                                               // Condição: verifica se o pino do relé atual é inválido (-1 no array 'reles').
    indiceReleAtual++;                                                                // Incrementa índice: avança para o próximo canal.
    tentativasAtual = 0;                                                              // Reseta contador: prepara para novas tentativas no próximo canal.
    return;                                                                           // Encerra a execução da função: pula este canal inválido.
  }

  // Lógica para iniciar a irrigação:
  if (!irrigando && !aguardandoProximo && umidade <= LIMIAR_SECO) {                   // Condição: não está irrigando nem esperando E umidade está abaixo do limite.
    digitalWrite(pinoRele, HIGH);                                                     // Atua: liga o relé do canal atual (ativando a bomba/válvula).
    irrigando = true;                                                                 // Atualiza estado: marca que a irrigação está ativa.
    tempoInicioIrrigacao = agora;                                                     // Marca tempo: registra o início desta irrigação.
    ultrapassouLimiteSeguranca = false;                                               // Reseta flag: garante que a segurança não está ativa para esta nova irrigação.

    Serial.print("Rele ativado: ");                                                   // Saída serial: informa qual relé foi ativado.
    Serial.println(nomesReles[indiceReleAtual]);                                      // Saída serial: exibe o nome legível do relé.
    SerialBT.print("Rele ativado: ");                                                 // Saída Bluetooth: informa qual relé foi ativado.
    SerialBT.println(nomesReles[indiceReleAtual]);                                    // Saída Bluetooth: exibe o nome legível do relé.

    lcd.clear();                                                                      // limpa o Display
    lcd.setCursor(0, 0);                                                              // coluna e linha
    lcd.print("Rele ativado: ");                                                      // (Máximo de caracteres: 16)
    lcd.setCursor(0, 1);                                                              //
    lcd.print(nomesReles[indiceReleAtual]);                                           // 
  }

  // Lógica para parar a irrigação:
  if (irrigando) {                                                                    // Condição: verifica se a irrigação está atualmente ativa.
    // Condição: verifica se o tempo padrão de irrigação OU o tempo máximo de segurança foi atingido.
    if (agora - tempoInicioIrrigacao >= tempoIrrigando || agora - tempoInicioIrrigacao >= tempoMaximoIrrigacao) {
      if (agora - tempoInicioIrrigacao >= tempoMaximoIrrigacao) {                     // Lógica de segurança: verifica se o tempo máximo foi excedido.
        ultrapassouLimiteSeguranca = true;                                            // Atualiza flag: indica que a segurança foi ativada.
        Serial.println("Tempo máximo de irrigação excedido! Segurança ativada.");     // Saída serial: alerta sobre a segurança.
        SerialBT.println("Tempo máximo de irrigação excedido! Segurança ativada.");   // Saída Bluetooth: alerta sobre a segurança.

        lcd.clear();                                                                  // limpa o Display
        lcd.setCursor(0, 0);                                                          // coluna e linha
        lcd.print("Tempo maximo: ");                                                  // (Máximo de caracteres: 16)
        lcd.setCursor(0, 1);                                                          //
        lcd.print("irrigacao acabou");                                                //
      }

      digitalWrite(pinoRele, LOW);                                          // Atua: desliga o relé do canal atual.
      irrigando = false;                                                    // Atualiza estado: marca que a irrigação não está mais ativa.
      aguardandoProximo = true;                                             // Atualiza estado: indica que está esperando para o próximo ciclo/canal.
      tempoInicioEspera = agora;                                            // Marca tempo: registra o início da espera entre relés.

      Serial.print("Rele desativado: ");                                    // Saída serial: informa qual relé foi desativado.
      Serial.println(nomesReles[indiceReleAtual]);                          // Saída serial: exibe o nome legível do relé.
      SerialBT.print("Rele desativado: ");                                  // Saída Bluetooth: informa qual relé foi desativado.
      SerialBT.println(nomesReles[indiceReleAtual]);                        // Saída Bluetooth: exibe o nome legível do relé.

      lcd.clear();                                                          // limpa o Display
      lcd.setCursor(0, 0);                                                  // coluna e linha
      lcd.print("Rele desativado ");                                        // (Máximo de caracteres: 16)
      lcd.setCursor(0, 1);                                                  //
      lcd.print(nomesReles[indiceReleAtual]);                               //
    }
  }

  // Lógica para reavaliação da umidade ou avanço de canal após irrigação:
  if (!irrigando && !aguardandoProximo) {                                   // Condição: não está irrigando nem esperando.
    // Condição: umidade ainda abaixo do desejado E não atingiu limite de segurança.
    if (mediasPercentual[indiceReleAtual] < umidadeDesejada && !ultrapassouLimiteSeguranca) {
      tentativasAtual++;                                                    // Incrementa contador: registra mais uma tentativa para este canal.
      if (tentativasAtual < maxTentativas) {                                // Condição: verifica se ainda há tentativas restantes.
        Serial.print("Tentativa ");                                         // Saída serial: informa o número da tentativa.
        Serial.print(tentativasAtual);
        Serial.println(" falhou, reirrigando...");                          // Saída serial: indica que tentará irrigar novamente.
        
        lcd.clear();                                                        // limpa o Display
        lcd.setCursor(0, 0);                                                // coluna e linha
        lcd.print("Tentativa falhou");                                      // (Máximo de caracteres: 16)
        lcd.setCursor(0, 1);                                                //
        lcd.print(tentativasAtual);                                         //

        aguardandoProximo = false;                                          // Reseta estado: pronto para re-iniciar a irrigação.
      } else {                                                              // Condição: limite de tentativas para o canal foi atingido.
        Serial.print("Planta ");                                             // Saída serial: informa que o canal atingiu o limite.
        Serial.print(indiceReleAtual);
        Serial.println(" atingiu o limite de tentativas. Indo para o próximo.");

        lcd.clear();                                                        // limpa o Display
        lcd.setCursor(0, 0);                                                // coluna e linha
        lcd.print("Limite de agua");                                        // (Máximo de caracteres: 16)
        lcd.setCursor(0, 1);                                                //
        lcd.print("Proximo Planta");                                        //

        indiceReleAtual++;                                                  // Incrementa índice: avança para o próximo canal.
        tentativasAtual = 0;                                                // Reseta contador: zera tentativas para o novo canal.
        ultrapassouLimiteSeguranca = false;                                 // Reseta flag: garante que não afetará o próximo canal.
      }
    } else {                                                                // Condição: umidade desejada atingida ou segurança ativada.
      Serial.print("Umidade desejada atingida no canal ");                  // Saída serial: informa sucesso no canal.
      Serial.println(indiceReleAtual);

      lcd.clear();                                                          // limpa o Display
      lcd.setCursor(0, 0);                                                  // coluna e linha
      lcd.print("Umidade atingida");                                        // (Máximo de caracteres: 16)
      lcd.setCursor(0, 1);                                                  //
      lcd.print(indiceReleAtual);                                           //

      indiceReleAtual++;                                                    // Incrementa índice: avança para o próximo canal.
      tentativasAtual = 0;                                                  // Reseta contador: zera tentativas para o novo canal.
      ultrapassouLimiteSeguranca = false;                                   // Reseta flag: limpa o estado de segurança.
    }
  }

  // Lógica para finalizar o período de espera entre relés:
  if (aguardandoProximo && agora - tempoInicioEspera >= tempoEntreRele) {   // Condição: está esperando E o tempo de espera terminou.
    aguardandoProximo = false;                                              // Reseta estado: sistema não está mais esperando, pronto para próxima ação.
  }

  // Lógica para pular irrigação se a umidade já estiver suficiente:
  if (!irrigando && !aguardandoProximo && umidade > LIMIAR_SECO) {          // Condição: não está irrigando, não esperando E umidade já é alta.
    Serial.println("Irrigação não iniciada: umidade já suficiente.");       // Saída serial: informa que a irrigação não foi necessária.
    Serial.print("Planta ");                                                // Saída serial: mostra o canal e sua umidade.
    Serial.print(indiceReleAtual);
    Serial.print(" umidade: ");
    Serial.print(umidade);
    Serial.println("%");
    
    lcd.clear();                                                            // limpa o Display
    lcd.setCursor(0, 0);                                                    // coluna e linha
    lcd.print("Umidade Boa na ");                                           // (Máximo de caracteres: 16)
    lcd.setCursor(0, 1);                                                    //
    lcd.print(indiceReleAtual);                                             //

    indiceReleAtual++;                                                      // Incrementa índice: avança para o próximo canal.
    tentativasAtual = 0;                                                    // Reseta contador: zera tentativas para o novo canal.
  }
}

// ==============================
// FUNÇÃO: REMOVER ÁGUA QUENTE DO CANO
// ==============================

void removerAguaQuente() {
  if (esvaziamentoFeito) return;                                             // Garante que o esvaziamento só será executado uma vez por noite

  if (!esvaziandoCano) {                                                     // Se ainda não estiver esvaziando, inicia o processo
    digitalWrite(RELE_PINO_X, HIGH);                                              // Ativa o relé que esvazia o cano
    Serial.println("Esvaziando cano: Guardando a água quente!");
    SerialBT.println("Esvaziando cano: Guardando a água quente!");

    lcd.clear();                                                             // limpa o Display
    lcd.setCursor(0, 0);                                                     // coluna e linha
    lcd.print("Esvaziando cano");                                            // (Máximo de caracteres: 16)
    lcd.setCursor(0, 1);                                                     //
    lcd.print("Guardando a agua");                                           //

    tempoRemoverAgua = millis();                                             // Marca o tempo de início
    esvaziandoCano = true;                                                   // Sinaliza que o processo começou

  } else if (millis() - tempoRemoverAgua >= tempoEsvaziamento) {             // Verifica se o tempo de esvaziamento foi atingido
    digitalWrite(RELE_PINO_X, LOW);                                               // Desliga o relé após 2 minutos
    Serial.println("Sistema vazio. Agora pode começar a irrigar.");     
    SerialBT.println("Sistema vazio. Agora pode começar a irrigar.");

    lcd.clear();                                                             // limpa o Display
    lcd.setCursor(0, 0);                                                     // coluna e linha
    lcd.print("Sistema vazio");                                              // (Máximo de caracteres: 16)
    lcd.setCursor(0, 1);                                                     //
    lcd.print("comecar irrigar");                                            //

    esvaziandoCano = false;                                                  // Marca fim do processo
    esvaziamentoFeito = true;                                                // Garante que não será executado de novo hoje
  }
}

// ==============================
// FUNÇÃO: SONO PROFUNDO ESP32
// ==============================

void modoSonoEsp32() {
  // Etapa 1: Verifica se o processo de preparo para sono já foi iniciado
  if (!preparandoSono) {
    SerialBT.println("Entrando em modo sono profundo por 10 horas...");
    Serial.println("Entrando em modo sono profundo por 10 horas...");

    lcd.clear();                                                            // limpa o Display
    lcd.setCursor(0, 0);                                                    // coluna e linha
    lcd.print("Sono Profundo");                                             // (Máximo de caracteres: 16)
    lcd.setCursor(0, 1);                                                    //
    lcd.print("Por 10 horas");                                              //

    tempoInicioPreSono = millis();                                          // Serve como marcador de tempo de início da preparação para o sono.
    preparandoSono = true;                                                  // Evita que o código dentro do if (!preparandoSono) rode novamente a cada loop.
  }

  // Etapa 2: Aguarda 2 segundos (2000 ms) de tolerância antes de iniciar o sono
  if (preparandoSono && millis() - tempoInicioPreSono >= 2000) {
    
    // 10 horas * 60 minutos * 60 segundos * 1.000.000 microssegundos =  36.000.000.000 microssegundos (36 bilhões). essa função espera um valor em microssegundos
    const uint64_t TEMPO_SONO_MICROS = 10ULL * 60ULL * 60ULL * 1000000ULL;  // Define o tempo de sono profundo (em microssegundos)

    // Garante que todos os relés estejam desligados por segurança antes de dormir
    // Desliga o relé de esvaziamento. lembrando que o rele x talvez não entre no for, dependendo se você não for usar ele no multiplexador. 
    digitalWrite(RELE_PINO_X, LOW);            

    digitalWrite(RELE_PINO_A, LOW);                                         // Desliga o relé A
    digitalWrite(RELE_PINO_B, LOW);                                         // Desliga o relé B
    /*
    for (int i = 0; i < numeroPino; i++) {                                  // Caso adicione mais relés futuramente, prefira usar um for
      if (reles[i] != -1) {
        digitalWrite(reles[i], LOW);
      }
    }
    */
    esp_sleep_enable_timer_wakeup(TEMPO_SONO_MICROS);                       // Configura o temporizador para acordar em 6 horas
    esp_deep_sleep_start();                                                 // Entra em sono profundo (deep sleep)
  }
}

// =====================================
// FUNÇÃO PARA LER UM CANAL DO MULTIPLEXADOR (NÃO BLOQUEANTE)
// =====================================

int lerMultiplexador() {                                        // Configura o multiplexador para o canalAtual global e lê a entrada analógica no pino SIG    
    for (int b = 0; b < 4; b++) {                               // Loop para definir os estados (HIGH/LOW) dos pinos seletores (S0, S1, S2, S3) de acordo com a 'tabelaMUX' e o 'canalAtual'.
        digitalWrite(seletores[b], tabelaMUX[canalAtual][b]);   // Seleciona o canal atual no multiplexador (usando a variável global canalAtual)
    }
    return analogRead(SIG);                                     // Retorna o valor da leitura analógica do canal selecionado
}

// =====================================
// FUNÇÃO PARA COLETAR E ACUMULAR LEITURAS (NÃO BLOQUEANTE) 
// =====================================

// Lê o valor do canal atual, acumula na soma e avança para o próximo canal
void coletarLeituras() {
       
        int valorLido = lerMultiplexador();         // Lê o valor analógico do canal atual
        valores_analogicos[canalAtual] = valorLido; // Armazena leitura para debug/monitoramento
        somaLeituras[canalAtual] += valorLido;      // Soma o valor na variável acumuladora do canal
        totalAmostrasJanela++;                      // Incrementa o contador total de leituras realizadas nesta janela de 60s 
        
        canalAtual++;                               // Avança para o próximo canal
        if (canalAtual >= 16) {                     // Se passou do último canal
            canalAtual = 0;                         // Volta para o canal 0 após ler o canal 15 (inicia um novo ciclo de 16 canais)
        }
}

// =====================================
// FUNÇÃO DE MÉDIA DE ACUMULADORES DE LEITURAS 
// =====================================

// Calcula a média inteira das leituras acumuladas para cada canal na janela de 60 segundos.
void calcularMedia60s() {
    
    int amostrasPorCanal = 0;
    if (totalAmostrasJanela > 0) {                          // totalAmostrasJanela é o número total de leituras individuais (que será um múltiplo de 16).
        amostrasPorCanal = totalAmostrasJanela / 16;        // Total de leituras / número de canais = amostras por canal
    }

    if (amostrasPorCanal > 0) {                             // Verifica se há pelo menos uma amostra completa por canal para evitar divisão por zero.
      for (int i = 0; i < 16; i++) {                        // Loop para calcular a média para cada um dos 16 canais
          medias[i] = somaLeituras[i] / amostrasPorCanal;   // Realiza a divisão inteira (não usar decimais)
        }
    } else {                                                // Mensagem de erro caso não haja dados suficientes para calcular a média (não deve ocorrer em operação normal)
        Serial.println("===== ERRO: Sem amostras coletadas no ciclo anterior para calcular média =====");
        SerialBT.println("===== ERRO: Sem amostras coletadas =====");
    }
}

// =====================================
// FUNÇÃO PARA DEIXAR OS VALORES EM PORCENTAGEM 
// =====================================

// Esta função converte as leituras médias (valores brutos do sensor) em uma porcentagem
void percentualLeituras() {                         
  for (int pino = 0; pino < numeroPino; pino++) {    // Loop para processar cada um dos pinos (canais) do multiplexador.
    int leitura = medias[pino];                    // Lê o valor médio já estável para o pino especificado
    int minimo = sensoresMinimos[pino];            // valor 4095 - Valor mínimo aprendido. Pega os limites mínimos registrados para o sensor
    int maximo = sensoresMaximos[pino];            // Valor 0 - Valor máximo aprendido. Pega os limites máximos registrados para o sensor

    // Evita divisão por zero, em muitos sistemas embarcados, uma divisão por zero pode levar a: Travar o Microcontrolador, Reset Inesperado, Resultados Imprevisíveis.
    if (maximo == minimo) {                       // Este bloco de código atua como uma proteção. Ele verifica a condição antes que a divisão aconteça.
      mediasPercentual[pino] = 0;                 // Neste caso, o percentual é zero para evitar erros.
      continue;                                   // Pula para a próxima iteração do loop (próximo pino). Isso garante que a linha com map() que causaria o erro não seja executada para esse pino específico.
    }

    // Atenção: solo molhado = menor leitura → 100%  ||  solo seco = maior leitura → 0%     
    int percentual = map(leitura, minimo, maximo, 100, 0);      // Usa map() para converter a leitura para um valor entre 0 e 100
    mediasPercentual[pino] = constrain(percentual, 0, 100);     // Usa constrain() para garantir que o valor esteja entre 0% e 100%
  }
}

// =====================================
// FUNÇÃO DE EXIBIÇÃO DAS MÉDIAS
// =====================================

// Exibe as médias calculadas (que estão no array 'medias')
void exibirLeituras() {
    Serial.println("===== UMIDADE (%) =====");                                          // Imprime um cabeçalho indicando o que está sendo exibido
    SerialBT.println("===== UMIDADE (%) =====");
    
    for (int i = 0; i < 16; i++) {                                                      // Loop para imprimir a média de cada canal
        String msg = "Canal " + String(i) + ": " + String(mediasPercentual[i]) + " %";  // concatena o valor da média de cada canal com a palatra canal e armazena em msg. 
        Serial.println(msg);                                                            // Exibe o valor do array medias, que foi calculado por calcularMedia60s
        SerialBT.println(msg);                                                          // Exibe o valor do array medias, que foi calculado por calcularMedia60s
    }
    Serial.println();                                                                   // Imprime uma linha em branco para separar as exibições
    SerialBT.println();                                                                 // Quebra de linha final
    
    // Exibição no LCD usando millis (não bloqueante)
    unsigned long agora = millis();

    if (agora - ultimaTrocaLCD >= intervaloLCD) {
        ultimaTrocaLCD = agora; // Atualiza o tempo da última troca

        lcd.clear();

        // Linha 1: Pino indiceLCD
        lcd.setCursor(0, 0);
        lcd.print("P");
        lcd.print(indiceLCD);
        lcd.print(": ");
        lcd.print(mediasPercentual[indiceLCD]);
        lcd.print("%");

        // Linha 2: Pino indiceLCD + 1 (se válido)
        if (indiceLCD + 1 <= 16) {
            lcd.setCursor(0, 1);
            lcd.print("P");
            lcd.print(indiceLCD + 1);
            lcd.print(": ");
            lcd.print(mediasPercentual[indiceLCD + 1]);
            lcd.print("%");
        }
        // Avança para o próximo par
        indiceLCD++;
        if (indiceLCD > 15) { // Último par é 15 e 16
            indiceLCD = 0; // Reinicia
        }
    }
}

// =====================================
// FUNÇÃO PARA EXIBIR MÉDIA DE PINO ESPECÍFICO 
// =====================================

// Imprime a média calculada para um canal (pino) específico no serial e via Bluetooth.
void imprimirMediaPinoEspecifico(byte pino) {                                                 // Recebe o número do pino (0-15) como argumento.
      
      if (pino >= 0 && pino < 16) {                                                           // Verifica se o número do pino está dentro do intervalo válido (0 a 15)
        String msg = "Pino " + String(pino) + ": " + String(mediasPercentual[pino]) + " %";   // Constrói a mensagem com a média do pino especificado
        Serial.println(msg);                                                                  // Imprime a mensagem no Monitor Serial
        SerialBT.println(msg);                                                                // Envia a mensagem via Bluetooth  
        // Exibe no LCD
        lcd.clear(); // Limpa a tela para a nova informação
        lcd.setCursor(0, 0); // Vai para o início da primeira linha
        lcd.print("Pino ");
        lcd.print(pino);
        lcd.print(" Umidade:");

        lcd.setCursor(0, 1); // Vai para o início da segunda linha
        lcd.print(mediasPercentual[pino]);
        lcd.print("%");
    } else {
        String msg = "Erro: Pino " + String(pino) + " invalido. Use um valor entre 0 e 15.";  // Mensagem de erro se o pino for inválido
        Serial.println(msg);
        SerialBT.println(msg);
        // Exibe erro no LCD também
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ERRO PINO!");
        lcd.setCursor(0, 1);
        lcd.print("Pino ");
        lcd.print(pino);
        lcd.print(" invalido.");
    }
}

// =====================================
// FUNÇÃO PARA ZERAR ACUMULADORES
// =====================================

// Limpa as variáveis que armazenam as somas e a contagem de amostras. Preparando-as para o próximo ciclo de cálculo de média.
void zerarTudoFree() {
    for (int i = 0; i < 16; i++) {                                // Loop para zerar a soma acumulada de cada canal
        valores_analogicos[i] = 0;                                // Zera o valor bruto para todos os canais
        somaLeituras[i] = 0;                                      // Zera a soma para todos os canais
        medias[i] = 0;                                            // Zera a média para todos os canais
        mediasPercentual[i] = 0;                                  // Zera o valor percentual para todos os canais
    }
    
    totalAmostrasJanela = 0;                                      // Zera o contador total de amostras de leituras para a próxima janela de tempo
    // Serial.println("--- Acumuladores Zerados ---");            // Pode descomentar para debug
}

// ======================
// Função para ler comandos da planilha
// ======================

void lerComandosDaPlanilha() {
  comandoLed = lerCelula("dados", "M2");    // Lê o comando do LED da célula M2 da aba "dados"
  comandoLed.trim();                        // Remove espaços em branco no início e fim da string
  comandoReleA = lerCelula("dados", "N2");  // Lê o comando do relé A da célula N2 da aba "dados"
  comandoReleA.trim();                      // Remove espaços em branco
  comandoReleB = lerCelula("dados", "O2");  // Lê o comando do relé B da célula O2 da aba "dados"
  comandoReleB.trim();                      // Remove espaços em branco
  // trim(): para eliminar espaços em branco que podem vir da planilha, evitando problemas em comparações.

  // Exibe no monitor serial os comandos lidos
  Serial.println("Comando do LED: " + comandoLed);
  Serial.println("Comando do RELE A: " + comandoReleA);
  Serial.println("Comando do RELE B: " + comandoReleB);
}

// ======================
// Atualiza conforme comando ou estado da nuvem
// ======================

void atualizarValoresPlanilhaNuvem() {

  // ************************************************* Planilha *************************************************
  // Determina o estado desejado do LED e dos relés com base nos comandos lidos da planilha ou nos flags locais (led_teste, releA, releB)
  estadoDesejadoLed = (comandoLed == "1" || led_teste == true);
  estadoDesejadoReleA = (comandoReleA == "1" || releA == true);
  estadoDesejadoReleB = (comandoReleB == "1" || releB == true);

  // LED e Relés com função genérica
  // Nome do pino - Nome no Monitor - Estado atual - valor declarado global
  atualizarEstado(ledpin, "Led ESP32", estadoDesejadoLed, estadoLedAtual);
  atualizarEstado(RELE_PINO_A, "Rele A", estadoDesejadoReleA, estadoReleA);
  atualizarEstado(RELE_PINO_B, "Rele B", estadoDesejadoReleB, estadoReleB);

  // === Sensor DHT11 ===
  temperatura = mediaTemp; // Para testes utilizei: random(35, 41); 
  umidadeAr =  mediaUmid; // Para testes utilizei: random(60, 90); 

  // === LDR ===
  luminosidade = mediaLuz; // Para testes utilizei: random(0, 100); 

  // === Umidade do solo ===
  umidadeSolo3 = mediasPercentual[3];   // Para testes utilizei: random(0, 100);   
  umidadeSolo10 = mediasPercentual[10]; // Para testes utilizei: random(0, 100);  
  umidadeSolo14 = mediasPercentual[14]; // Para testes utilizei: random(0, 100);  

  // ************************************************* Nuvem *************************************************

  // ========= LED =========
  onLedTesteChange();  // <--- Essa função já está enviado o valor do led
  // ======== Relés ========
  onReleAChange();  // <--- Essa função já está enviado o acionamento do Rele
  onReleBChange();  // <--- Essa função já está enviado o acionamento do Rele
  // ===== Sensor DHT11 ====
  onTemperaturaNuvemChange();  // <--- Essa função já está enviado o valor de temperatura
  onUmidadeNuvemChange();      // <--- Essa função já está enviado o valor de umidade
  // ========= LDR =========
  onLuminosidadeNuvemChange();  // <--- Essa função já está enviado o valor de Luminosidade
  // === Umidade do solo ===
  onUmidadeSoloNuvem3Change();   // <--- Essa função já está enviado o valor de Umidade do solo do canal 3
  onUmidadeSoloNuvem10Change();  // <--- Essa função já está enviado o valor de Umidade do solo do canal 10
  onUmidadeSoloNuvem14Change();  // <--- Essa função já está enviado o valor de Umidade do solo do canal 14
}

// ======================
// Atualiza os sensores digitais como led e reles que são dados: 0 (Desligado) e 1 (Ligado).
// ======================

void atualizarEstado(int pino, String nome, bool estadoDesejado, float& estadoAtual) {
  // Um ponto importante, a planilha não controla a DashBoard do Arduino Cloud ao inserir comandos lá por meio de lerCelula(), já que quem prevalece
  // é o dashboard que é só o que o usuario iria usar. Para que a planilha controlasse, eu proponho a mudança abaixo: Atualizar a variáve e a célula.
  if (estadoDesejado) {
    digitalWrite(pino, HIGH);
    // Se for verdadeira: led_teste == true
    // celula escolhidad para dar o comando vai ser inserido o numero 1.
    estadoAtual = 1.0;  // LED está ligado.
    Serial.println(nome + " - Ligado");
  } else {
    digitalWrite(pino, LOW);
    // Se for falso: led_teste == false
    // celula escolhidad para dar o comando vai ser inserido o numero 0.
    estadoAtual = 0.0;  // LED está desligado
    Serial.println(nome + " - Desligado");
  }
}
// ======================
// Enviando todos os dados para a planilha
// ======================

void registrarDadosPlanilha() {

  float dadosParaEnviar[9] = {
    // Prepara um vetor com os dados que serão enviados para a planilha do Google Sheets
    estadoLedAtual,  // Estado atual do LED (ligado/desligado)
    estadoReleA,     // Estado atual do relé A (ligado/desligado)
    estadoReleB,     // Estado atual do relé B (ligado/desligado)
    temperatura,     // Valor atual da temperatura lida pelo sensor
    umidadeAr,       // Valor atual da umidade do ar lida pelo sensor
    luminosidade,    // Valor atual da luminosidade lida pelo sensor LDR
    umidadeSolo3,    // Umidade do solo medida no sensor do canal 3
    umidadeSolo10,   // Umidade do solo medida no sensor do canal 10
    umidadeSolo14,   // Umidade do solo medida no sensor do canal 14
  };

  escreverEmLista("dados", 9, dadosParaEnviar);  // Envia o dado para a aba "dados" da planilha
}

// ======================
// Utilitários de envio e leitura HTTP
// ======================

String seguirRedirecionamento(String url) {
  HTTPClient http;            // Cria um objeto para realizar requisições HTTP
  http.begin(url);            // Inicia a requisição com a URL fornecida
  int httpCode = http.GET();  // Faz uma requisição HTTP GET e armazena o código de resposta

  if (httpCode == HTTP_CODE_MOVED_PERMANENTLY || httpCode == HTTP_CODE_FOUND) {
    String newUrl = http.getLocation();     // Obtém a nova URL do redirecionamento
    http.end();                             // Encerra a conexão HTTP atual
    return seguirRedirecionamento(newUrl);  // Chama recursivamente a função para seguir o redirecionamento
  } else {
    String response = http.getString();  // Lê a resposta da requisição HTTP
    http.end();                          // Encerra a conexão HTTP
    return response;                     // Retorna o conteúdo da resposta
  }
}
// ======================
// envia um conjunto de dados (vetor de float) em formato JSON para uma planilha online
// ======================

bool escreverEmLista(String identificacao, int numDados, float dados[]) {
  bool flag_envio = 0;                                 // Flag que indica se o envio foi bem-sucedido
  String url = googleScriptURL;                        // URL do script Google Apps (planilha online)
  HTTPClient http;                                     // Cria um cliente HTTP
  http.begin(url);                                     // Inicia a conexão com a URL
  http.addHeader("Content-Type", "application/json");  // Define o cabeçalho para envio de dados em formato JSON

  StaticJsonDocument<200> jsonDoc;           // Documento JSON estático (sugestão: usar 512 se tiver muitos dados)
  jsonDoc["action"] = "escreverEmLista";     // Define a ação que o script deve executar
  jsonDoc["identificacao"] = identificacao;  // Define o identificador dos dados enviados

  JsonArray jsonDados = jsonDoc.createNestedArray("dados");  // Cria um array dentro do JSON para armazenar os dados
  for (int i = 0; i < numDados; i++) {
    jsonDados.add(dados[i]);  // Adiciona os dados individuais ao array
  }

  String jsonString;
  serializeJson(jsonDoc, jsonString);  // Converte o documento JSON em string

  int httpResponseCode = http.POST(jsonString);  // Envia a requisição POST com o JSON para o servidor

  if (httpResponseCode == HTTP_CODE_MOVED_PERMANENTLY || httpResponseCode == HTTP_CODE_FOUND) {
    String newUrl = http.getLocation();  // Se houve redirecionamento, obtém a nova URL
    http.end();                          // Finaliza a requisição atual
    seguirRedirecionamento(newUrl);      // Chama a função para seguir o redirecionamento
    Serial.println("Dados enviados");    // Mostra mensagem de sucesso no console
    flag_envio = 1;                      // Define flag de envio como verdadeiro
  } else if (httpResponseCode > 0) {
    String response = http.getString();  // Lê a resposta recebida (caso não seja redirecionamento)
    Serial.println(response);            // Imprime a resposta no console
  } else {
    Serial.println("Erro ao enviar dados");  // Mensagem de erro se não conseguir enviar
    flag_envio = 0;                          // Define flag de envio como falso
  }

  http.end();         // Encerra a conexão HTTP
  return flag_envio;  // Retorna se o envio foi bem-sucedido
}
// ======================
//  envia um dado específico para uma célula determinada de uma planilha Google Sheets via requisição HTTP em formato JSON
// ======================

bool escreverEmCelula(String identificacao, String celula, String dado) {
  bool flag_envio = 0;                                 // Flag para indicar se o envio foi bem-sucedido (1) ou não (0)
  String url = googleScriptURL;                        // URL do script Google Apps que irá receber os dados
  HTTPClient http;                                     // Cria um cliente HTTP
  http.begin(url);                                     // Inicia a conexão com a URL
  http.addHeader("Content-Type", "application/json");  // Define o cabeçalho da requisição como JSON

  StaticJsonDocument<200> jsonDoc;           // Documento JSON estático (pode aumentar para 512 se necessário)
  jsonDoc["action"] = "escreverEmCelula";    // Informa a ação que o script deve executar
  jsonDoc["identificacao"] = identificacao;  // Identificador usado para localizar o destino do dado
  jsonDoc["celula"] = celula;                // Localização da célula na planilha onde o dado será escrito
  jsonDoc["dado"] = dado;                    // Valor a ser inserido na célula especificada

  String jsonString;
  serializeJson(jsonDoc, jsonString);  // Converte o documento JSON para uma string a ser enviada

  int httpResponseCode = http.POST(jsonString);  // Envia a requisição POST com os dados JSON

  if (httpResponseCode == HTTP_CODE_MOVED_PERMANENTLY || httpResponseCode == HTTP_CODE_FOUND) {
    String newUrl = http.getLocation();  // Se o servidor responder com redirecionamento, pega a nova URL
    http.end();                          // Finaliza a requisição atual
    seguirRedirecionamento(newUrl);      // Segue o redirecionamento automaticamente
    Serial.println("Dados enviados");    // Informa sucesso no envio
    flag_envio = 1;                      // Marca o envio como bem-sucedido
  } else if (httpResponseCode > 0) {
    String response = http.getString();  // Lê a resposta da requisição (caso não seja redirecionamento)
    Serial.println(response);            // Exibe a resposta no console
  } else {
    Serial.println("Erro ao enviar dados");  // Exibe mensagem de erro se falhar o envio
    flag_envio = 0;                          // Marca o envio como mal-sucedido
  }

  http.end();         // Encerra a conexão HTTP
  return flag_envio;  // Retorna o status do envio
}
// ======================
// O código lê o valor de uma célula em uma planilha online e via requisição HTTP.
// ======================

String lerCelula(String identificacao, String celula) {
  String url = googleScriptURL;                        // Define a URL do script Google Apps responsável pela leitura
  HTTPClient http;                                     // Cria uma instância do cliente HTTP
  http.begin(url);                                     // Inicia a conexão com a URL especificada
  http.addHeader("Content-Type", "application/json");  // Define o cabeçalho como JSON para a requisição POST

  StaticJsonDocument<200> jsonDoc;           // Documento JSON estático (200 bytes). Pode ser aumentado para 512 se necessário
  jsonDoc["action"] = "lerCelula";           // Define a ação que o script do Google deve executar
  jsonDoc["identificacao"] = identificacao;  // Identificador da planilha ou contexto da leitura
  jsonDoc["celula"] = celula;                // Endereço da célula que será lida (ex: "B2")

  String jsonString;
  serializeJson(jsonDoc, jsonString);  // Serializa o JSON para uma string que será enviada

  int httpResponseCode = http.POST(jsonString);  // Envia a requisição POST e armazena o código de resposta

  if (httpResponseCode == HTTP_CODE_MOVED_PERMANENTLY || httpResponseCode == HTTP_CODE_FOUND) {
    String newUrl = http.getLocation();     // Se houver redirecionamento, obtém a nova URL
    http.end();                             // Encerra a conexão atual
    return seguirRedirecionamento(newUrl);  // Chama a função para seguir o redirecionamento e retorna o resultado
  } else if (httpResponseCode > 0) {
    String response = http.getString();  // Se a resposta for válida, obtém a string de resposta
    http.end();                          // Encerra a conexão
    return response;                     // Retorna o conteúdo da célula lida
  } else {
    http.end();                   // Encerra a conexão em caso de erro
    return "Erro ao ler célula";  // Retorna mensagem de erro
  }
}
// ======================
// envia uma requisição para um Google Script para ler e retornar os dados de uma linha específica de uma planilha
// ======================

String lerLinha(String identificacao, int linha) {
  String url = googleScriptURL;                        // Define a URL do Google Script para a requisição
  HTTPClient http;                                     // Cria objeto HTTPClient para realizar a requisição HTTP
  http.begin(url);                                     // Inicia conexão com a URL especificada
  http.addHeader("Content-Type", "application/json");  // Define cabeçalho para indicar que os dados são JSON

  StaticJsonDocument<200> jsonDoc;           // Cria documento JSON estático para montar o corpo da requisição (pode aumentar para 512 se necessário)
  jsonDoc["action"] = "lerLinha";            // Define a ação a ser realizada no Google Script (ler uma linha)
  jsonDoc["identificacao"] = identificacao;  // Passa a identificação (ex: nome da planilha ou chave)
  jsonDoc["linha"] = linha;                  // Passa o número da linha que será lida

  String jsonString;
  serializeJson(jsonDoc, jsonString);  // Serializa o JSON para string, que será enviada via POST

  int httpResponseCode = http.POST(jsonString);  // Envia o POST e recebe o código HTTP de resposta

  if (httpResponseCode == HTTP_CODE_MOVED_PERMANENTLY || httpResponseCode == HTTP_CODE_FOUND) {
    String newUrl = http.getLocation();     // Se houver redirecionamento, obtém a nova URL
    http.end();                             // Encerra a conexão atual
    return seguirRedirecionamento(newUrl);  // Chama função recursiva para seguir o redirecionamento e retorna o resultado
  } else if (httpResponseCode > 0) {
    String response = http.getString();  // Se a resposta foi bem-sucedida, obtém a resposta em string
    http.end();                          // Encerra a conexão
    return response;                     // Retorna o conteúdo da linha lida
  } else {
    http.end();                  // Encerra a conexão em caso de erro
    return "Erro ao ler linha";  // Retorna mensagem de erro
  }
}
// ======================
// verifica se o cabeçalho da planilha está correto e, se não estiver, escreve os títulos das colunas na primeira linha, começando por uma coluna inicial especificada.
// ======================

void montarCabecalho(String _boardID, const String& colunaInicial, const std::vector<String>& cabecalhos) {

  String celula = lerCelula(_boardID, colunaInicial + "1");  // Verifica se a primeira célula da planilha já contém o primeiro cabeçalho esperado

  if (celula != cabecalhos[0] && celula != "Erro ao ler célula") {  // Se a célula lida for diferente do primeiro cabeçalho esperado e não for um erro de leitura
    Serial.print("Carregando cabeçalho...");
    Serial.println("");
    Serial.print("String recebida: ");
    Serial.println(celula);
    Serial.println("");

    char coluna = colunaInicial[0];  // Começa na coluna inicial (exemplo: 'A')

    for (size_t i = 0; i < cabecalhos.size(); i++) {  // Para cada item da lista de cabeçalhos
      String celulaAlvo = String(coluna) + "1";       // Define a célula alvo concatenando a coluna atual com o número da linha 1 (ex: A1, B1, C1...)
      while (escreverEmCelula(_boardID, celulaAlvo, cabecalhos[i]) == 0)
        ;        // Tenta escrever o cabeçalho na célula alvo até que seja bem-sucedido (escreverEmCelula retorna 1)
      coluna++;  // Avança para a próxima coluna (incrementa o caractere)

      if (coluna > 'Z') coluna = 'A';  // Caso ultrapasse a coluna 'Z', volta para 'A' (opcional, caso tenha mais de 26 colunas)
    }
  }
}

// =====================================
// FUNÇÃO: delay feita manualmente 
// =====================================

// Aguarda o tempo definido sem bloquear completamente o código
void aguardar(unsigned long tempo_ms) {
  unsigned long inicio = millis();
  while (millis() - inicio < tempo_ms);
}

// ==============================
// FUNÇÃO: indicador de atividade 
// ==============================

void atualizarIndicador(unsigned long tempoAtualMillis) {

  if (tempoAtualMillis - ultimoTempoIndicador >= intervaloIndicador) {
    ultimoTempoIndicador = tempoAtualMillis;

    contadorPontos++; //Essa variável parece ser o coração do indicador visual ( controlando o número de pontos que serão exibidos)
      if (contadorPontos > 5) { // Isso cria um ciclo, fazendo com que o número de pontos exibidos vá de 1 a 5 e depois volte para 1, repetidamente.
        contadorPontos = 1; // Se contadorPontos for maior que 5, ele é resetado para 1.
      }

    String pontos = ""; // Essa variável será usada para construir a sequência de pontos a serem exibidos no LCD.
    for (int i = 0; i < contadorPontos; i++) { // Este é um loop for que se repete contadorPontos vezes.
      pontos += "."; //Em cada iteração do loop, um caractere ponto (.) é adicionado à string pontos 
    }

    // Exibe no LCD - mova da direita para a esquerda na tela
    lcd.clear();
    lcd.setCursor(15 - contadorPontos, 1); // coluna e linha: Ajusta para caber na tela. Se contadorPontos for 5, o cursor será posicionado na coluna 10 (15 - 5).
    lcd.print(pontos + "     "); // Espaço extra para limpar pontos anteriores, especialmente se o número de pontos diminuir no ciclo.

    // Exibe no Monitor Serial
    Serial.println(pontos);

    // Exibe via Bluetooth
    SerialBT.println(pontos);
  }
}

// ======================
// Retorno de chamada da Nuvem IoT para controle e manipulação da Dashboard do Arduino IoT Cloud
// ======================

// Adicione seu código em cada função de manipulação da sua variavel escolhida para agir sobre a alteração da variavel da Nuvem no Arduino IoT Cloud.
// Como essas variaveis são de leitura e escrita, elas são executadas sempre que um novo valor é recebido da IoT Cloud.
void onLedTesteChange() {
  atualizarEstado(ledpin, "Led ESP32", estadoDesejadoLed, estadoLedAtual);
}

void onReleAChange() {
  atualizarEstado(RELE_PINO_A, "Rele A", estadoDesejadoReleA, estadoReleA);
}

void onReleBChange() {
  atualizarEstado(RELE_PINO_B, "Rele B", estadoDesejadoReleB, estadoReleB);
}

void onTemperaturaNuvemChange() {
  temperaturaNuvem = temperatura;
}

void onUmidadeNuvemChange() {
  umidadeNuvem = umidadeAr;
}

void onLuminosidadeNuvemChange() {
  luminosidadeNuvem = luminosidade;
}

void onUmidadeSoloNuvem3Change() {
  umidadeSoloNuvem_3 = umidadeSolo3;
}

void onUmidadeSoloNuvem10Change() {
  umidadeSoloNuvem_10 = umidadeSolo10;
}

void onUmidadeSoloNuvem14Change() {
  umidadeSoloNuvem_14 = umidadeSolo14;
}