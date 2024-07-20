/**
 *Este é um exemplo de código para um dispositivo Bluetooth de leitura de ECG (eletrocardiograma) baseado no
  módulo Bluefruit52.
 *O dispositivo realiza a leitura dos sinais ECG e envia os dados através de BLE UART.
 *Além disso, monitora a voltagem da bateria e possui recursos de gerenciamento de energia para otimizar o
  consumo.
 *O código também inclui a configuração do serviço OTA DFU para facilitar as atualizações de firmware.
 *Para iniciar, o dispositivo utiliza a biblioteca Bluefruit e suas funcionalidades.
 *
 * Autor: LSA
 * Atualizado por: LSA
 */


 #include <Arduino.h>
 #include <bluefruit.h>
 #include <Adafruit_LittleFS.h>
 #include <InternalFileSystem.h>
 #include <Adafruit_nRFCrypto.h>
 #include <sys/types.h>

 // Definição dos Serviços BLE
 BLEDfu bledfu; // Serviço OTA DFU
 BLEDis bledis; // Informações do dispositivo
 BLEUart bleuart; // Comunicação UART sobre BLE

 // Buffer para dados do ECG
 static uint32_t ecg[32] = {0};
 size_t pos = 0;
 bool isConnected = false;

 // Resolução do Conversor Analógico para Digital (ADC)
 #define ADC_RESOLUTION 12
 float adc_max;
 float adc_mean;

 #define V_MAX 3.0
 #define V_MIN 0.0

 float ecg_factor;
 float ecg_voltage;

 // Configuração da leitura da bateria
 #define BAT_READ_CNT (8)
 #define BAT_MAX_VOLTAGE 5000.0
 float bat_factor;
 float bat_voltage;
 uint32_t Vbatt = 0;

 // Pino de controle do SDN (Shutdown)
 #define SDN_PIN D8
 bool isActive = false;

 // Buffer para leituras do ECG
 #define LEITURAS 7500
 static uint32_t buffer[LEITURAS] = {0};
 static size_t leitura = 0;

 // Limite de tensão para bateria baixa (em mV)
 #define LOW_BATTERY_VOLTAGE 3500

 void setup() {
 analogReadResolution(ADC_RESOLUTION);
 adc_max = (1 << ADC_RESOLUTION) - 1;
 adc_mean = 1 << (ADC_RESOLUTION - 1);

 pinMode(11, INPUT);
 pinMode(12, INPUT);
 pinMode(SDN_PIN, OUTPUT);
 pinMode(13, OUTPUT); // Saída de LED
 Serial.begin(115200);
 sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

 // Configuração da bateria
 pinMode(PIN_VBAT, INPUT);
 bat_factor = (5000.0 / adc_max) / BAT_READ_CNT;

 // Configuração do cálculo do ECG
 ecg_factor = (V_MAX - V_MIN) / adc_max;

 Serial.println("Bluefruit52 BLEUART Example");
 Serial.println("---------------------------\n");

 Bluefruit.autoConnLed(true);
 Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

 Bluefruit.begin();
 Bluefruit.setTxPower(4);
 Bluefruit.Periph.setConnectCallback(connect_callback);
 Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

 Bluefruit.setName("Cardioleitor");

 bledfu.begin();

 bledis.setManufacturer("Adafruit Industries");
 bledis.setModel("Bluefruit Feather52");
 bledis.begin();

 bleuart.begin();

 Serial.print("UUID: ");
 Serial.println(bleuart.uuid.toString().c_str());

 startAdv();

 Serial.println("Please use Adafruit's Bluefruit LE app to connect in UART mode");
 Serial.println("Once connected, enter character(s) that you wish to send");
 Serial.print("Remember, max MTU is: ");
 Serial.println(Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH));

 // Piscar o LED para sinalizar o início
 for (int i = 0; i < 3; i++) {
 digitalWrite(13, LOW);
 delay(40);
 digitalWrite(13, HIGH);
 delay(100);
 }
 }

 void startAdv(void) {
 Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
 Bluefruit.Advertising.addTxPower();
 Bluefruit.Advertising.addService(bleuart);
 Bluefruit.ScanResponse.addName();

 Bluefruit.Advertising.restartOnDisconnect(true);
 Bluefruit.Advertising.setInterval(32, 244);
 Bluefruit.Advertising.setFastTimeout(30);
 Bluefruit.Advertising.start(0);

digitalWrite(13, LOW);
 delay(500);
 digitalWrite(13, HIGH);
 delay(500);
 digitalWrite(13, LOW);
 delay(500);
 digitalWrite(13, HIGH);


 }

 void loop() {
 leitura = 0;

 digitalWrite(SDN_PIN, HIGH);
 delay(1000); // Ativar o AD8232

 // Se os eletrodos forem desconectados no meio do processo
 // envie uma mensagem via BLE com uma string (Leads Off Detect Positivo/Negativo)

 while (leitura < LEITURAS) {
 buffer[leitura++] = analogRead(A0);
 delay(4);
 }

 Vbatt = 0;
 for (int i = 0; i < BAT_READ_CNT; i++) {
 Vbatt += analogRead(PIN_VBAT); // Lê o valor analógico de A0
 }
 bat_voltage = Vbatt * bat_factor; // Converte o valor lido para mV
 Serial.print("Tensão da bateria: ");
 Serial.print(bat_voltage, 3); // Imprime a tensão da bateria em mV com 3 casas decimais
 Serial.println(" mV");

 digitalWrite(SDN_PIN, LOW);
 digitalWrite(13, LOW);
 delay(40);
 digitalWrite(13, HIGH);
 leitura = 0;

 while (leitura < LEITURAS) {
 while (pos < 32 && leitura < LEITURAS) {
 ecg_voltage = ((float)buffer[leitura++] - adc_mean) * ecg_factor;
 Serial.println(ecg_voltage);
 ecg[pos++] = __htonl(*(unsigned int *)&ecg_voltage);
 }
 if (isConnected) {
 bleuart.write((const uint8_t *)&ecg, pos * 4);
 pos = 0;
 }
 }

 digitalWrite(13, LOW);
 delay(40);
 digitalWrite(13, HIGH);
 delay(100);
 digitalWrite(13, LOW);
 delay(40);
 digitalWrite(13, HIGH);

 delay(30000); // Intervalo de 30 segundos
 
 }

 void connect_callback(uint16_t conn_handle) {
 BLEConnection *connection = Bluefruit.Connection(conn_handle);
 isConnected = true;
 pos = 0;

 char central_name[32] = {0};
 connection->getPeerName(central_name, sizeof(central_name));

 Serial.print("Connected to ");
 Serial.print(central_name);
 Serial.print(" with MTU ");
 Serial.println(connection->getMtu());
 }

 void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
(void)conn_handle;
 (void)reason;

 isConnected = false;

 Serial.println();
 Serial.print("Disconnected, reason = 0x");
 Serial.println(reason, HEX);
 
sd_app_evt_wait();
 }