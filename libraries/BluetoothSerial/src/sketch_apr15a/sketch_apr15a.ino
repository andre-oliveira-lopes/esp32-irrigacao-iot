#include "BluetoothSerial.h"

// verifica se o bluettof está ligado corretamente 
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200); // Para visualizar no monitor serial
  delay(1000);          // Dá um tempo pro sistema estabilizar

  if (!SerialBT.begin("ESP32_BT")) {
    Serial.println("Erro ao iniciar Bluetooth");
  } else {
    Serial.println("Bluetooth pronto. Emparelhe com 'ESP32_BT'");
  }
}

void loop() {
  SerialBT.println("Mensagem do ESP32 via Bluetooth");
  delay(2000); // Aguarda 2 segundos antes de enviar novamente
}
