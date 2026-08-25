// #include <Arduino.h>

// // Настройка пинов UART для вашей управляющей ESP32
// #define ESP_RX_PIN 16 
// #define ESP_TX_PIN 17 

// // Используем второй аппаратный сериал (Serial2)
// HardwareSerial EspSerial(2);

// void setup() {
//   Serial.begin(115200);
  
//   // Скорость ДОЛЖНА СОВПАДАТЬ с настройкой Serial модуля в Meshtastic
//   // Обычно это 115200 по умолчанию, либо выставленные вами 9600
//   EspSerial.begin(115200, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);
  
//   Serial.println("ESP32 Master готов к отправке в Meshtastic.");
// }

// void loop() {
//   delay(30000); 
//   // Отправляем строку в UART. Символ '\n' (println) сигнализирует 
//   // Meshtastic о завершении формирования сообщения.
//   EspSerial.println("TEST");
  
//   Serial.println("Строка 'TEST' отправлена на Heltec (Meshtastic)...");
  
//   // Не отправляйте данные слишком часто! Meshtastic — это Mesh-сеть,
//   // частая отправка быстро забьет эфир. Раз в 30 секунд — оптимально для тестов.
//   delay(30000); 
// }








#include <Arduino.h>

// Используем встроенный в Meshtastic движок Nanopb
#include <pb_encode.h>
#include <pb_decode.h>

// Автоматически определяем правильный путь к структурам Meshtastic
#if __has_include(<meshtastic/mesh.pb.h>)
  #include <meshtastic/mesh.pb.h>
#else
  #include <mesh.pb.h>
#endif

// Настройка пинов UART для ESP32
#define ESP_RX_PIN 16 
#define ESP_TX_PIN 17 

HardwareSerial EspSerial(2);

// ID целевой ноды (c1e3b1c4)
const uint32_t TARGET_NODE_ID = 0xC1E3B1C4; 

unsigned long loopCounter = 0;

void setup() {
  Serial.begin(115200);
  
  // В бинарном режиме PROTO Meshtastic работает строго на 115200 бод
  EspSerial.begin(115200, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);
  delay(1000);

  // Синхронизируем Serial-интерфейс стартовыми wake-байтами
  for(int i = 0; i < 4; i++) {
    EspSerial.write(0x94);
  }
  
  Serial.println("ESP32 Мастер (Встроенный API Meshtastic/Nanopb) запущен.");
}

void sendDirectMessage(const char* text, uint32_t targetId) {
  uint8_t buffer[256]; // Выделяем массив под бинарные данные пакета
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

  // Инициализируем корневую структуру контейнера ToRadio
  meshtastic_ToRadio toRadioPacket = meshtastic_ToRadio_init_zero;
  toRadioPacket.which_payload_variant = meshtastic_ToRadio_packet_tag; 

  // Прямое заполнение структуры MeshPacket внутри контейнера ToRadio
  toRadioPacket.packet.to = targetId;   // ID получателя (0xC1E3B1C4)
  toRadioPacket.packet.from = 0;        // 0 заставит ноду подставить свой собственный ID
  toRadioPacket.packet.want_ack = true; // Запрашиваем аппаратный ACK (подтверждение доставки)

  // Настройка полезной нагрузки сообщения (Decoded Data)
  toRadioPacket.packet.which_payload_variant = meshtastic_MeshPacket_decoded_tag;
  toRadioPacket.packet.decoded.portnum = meshtastic_PortNum_TEXT_MESSAGE_APP; // Порт 1: Обычный текст

  // Копируем текст сообщения "TEST" в массив payload
  size_t text_len = strlen(text);
  if (text_len > sizeof(toRadioPacket.packet.decoded.payload.bytes)) {
    text_len = sizeof(toRadioPacket.packet.decoded.payload.bytes);
  }
  memcpy(toRadioPacket.packet.decoded.payload.bytes, text, text_len);
  toRadioPacket.packet.decoded.payload.size = text_len;

  // Кодируем структуру ToRadio встроенными средствами Nanopb
  if (!pb_encode(&stream, meshtastic_ToRadio_fields, &toRadioPacket)) {
    Serial.print("Ошибка упаковки Protobuf: ");
    Serial.println(PB_GET_ERROR(&stream));
    return;
  }

  size_t packet_length = stream.bytes_written;

  // Кадрирование бинарного пакета по стандарту Meshtastic Serial Framer
  // Отправляем маркеры начала [0x94] [0xC3] и 2 байта длины пакета
  EspSerial.write(0x94);
  EspSerial.write(0xC3);
  EspSerial.write((packet_length >> 8) & 0xFF);
  EspSerial.write(packet_length & 0xFF);
  
  // Передаем готовый бинарный Protobuf-пакет в Serial-шину Heltec
  EspSerial.write(buffer, packet_length);

  Serial.println("Бинарный DM пакет (ToRadio) успешно отправлен на ноду c1e3b1c4.");
}

void loop() {
  loopCounter++;
  String result = "TEST: " + String(loopCounter);
  // Отправка сообщения "TEST" лично для ноды c1e3b1c4
  sendDirectMessage(result.c_str(), TARGET_NODE_ID);
  
  // Интервал 30 секунд для предотвращения спама в Mesh-сети
  delay(30000); 
}
