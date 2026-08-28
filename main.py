import time
from pubsub import pub
import meshtastic
import meshtastic.serial_interface

# !!! УКАЖИТЕ ID НОДЫ СОБЕСЕДНИКА !!!
TARGET_NODE_ID = "f6fcea74"
TARGET_NODE_HEX = TARGET_NODE_ID.replace("!", "").lower()


def on_receive(packet, interface):
    try:
        # 1. Проверяем, что это текстовое сообщение
        if 'decoded' in packet and packet['decoded']['portnum'] == 'TEXT_MESSAGE_APP':

            # Получаем hex-ID отправителя
            sender_num = packet.get('from')
            sender_hex = f"{sender_num:08x}".lower() if sender_num else ""
            sender_from_id = packet.get('fromId', '').replace("!", "").lower()

            # Получаем ID получателя, чтобы понять, личное это сообщение или в канал
            recipient_num = packet.get('to')

            # Проверяем:
            # - Отправитель совпадает с TARGET_NODE_ID
            # - Получатель НЕ является широковещательным адресом (0xFFFFFFFF)
            is_from_target = TARGET_NODE_HEX in (sender_hex, sender_from_id)
            is_direct_message = recipient_num != 4294967295  # 4294967295 — это BROADCAST

            if is_from_target and is_direct_message:
                message_text = packet['decoded']['payload'].decode('utf-8')

                print(f"\n [Новое ЛИЧНОЕ сообщение от {TARGET_NODE_ID}]")
                print(f"Текст: {message_text}")
                print("-" * 35)

    except Exception as e:
        print(f"Ошибка обработки пакета: {e}")


# Подписка на события Meshtastic
pub.subscribe(on_receive, "meshtastic.receive")

print("Подключение к плате Heltec LoRa32 через USB...")
try:
    interface = meshtastic.serial_interface.SerialInterface()
    # interface = meshtastic.serial_interface.SerialInterface(devPath='COM16')
    print(f"Фильтр включен. Ожидаем только ЛИЧНЫЕ сообщения от: {TARGET_NODE_ID}")

    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nЗавершение работы...")
finally:
    if 'interface' in locals():
        interface.close()
