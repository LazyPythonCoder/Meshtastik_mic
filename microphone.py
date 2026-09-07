import serial
import wave
import sys
import time
import threading

# --- НАСТРОЙКИ ---
COM_PORT = 'COM21'  # Замените на номер вашего COM-порта
BAUD_RATE = 500000  # Скорость порта
OUTPUT_FILE = 'audio.wav'  # Имя итогового аудиофайла
SAMPLE_RATE = 16000  # Частота дискретизации

# Флаг для остановки записи
running = True


def check_keyboard():
    """Фоновая функция, которая ждет нажатия Enter"""
    global running
    input()  # Ждем, пока пользователь нажмет Enter
    running = False


def main():
    global running
    print(f"Подключение к порту {COM_PORT}...")
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    except Exception as e:
        print(f"Ошибка подключения к порту: {e}")
        return

    ser.flushInput()
    time.sleep(0.5)
    ser.flushInput()

    # Сначала создаем и открываем WAV файл в режиме записи байт
    wav_file = wave.open(OUTPUT_FILE, 'wb')
    wav_file.setnchannels(1)  # Моно
    wav_file.setsampwidth(2)  # 16-бит (2 байта)
    wav_file.setframerate(SAMPLE_RATE)

    print("\n=== ЗАПИСЬ НАЧАТА ===")
    print("Говорите в микрофон...")
    print("Чтобы ОСТАНОВИТЬ запись, нажмите [ENTER] в этом окне.")
    print("-" * 50)

    # Запускаем фоновый поток для отслеживания клавиатуры
    input_thread = threading.Thread(target=check_keyboard)
    input_thread.daemon = True
    input_thread.start()

    total_bytes = 0
    start_time = time.time()

    try:
        while running:
            # Проверяем, есть ли данные в буфере порта
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                if chunk:
                    # Сразу пишем данные на жесткий диск, не забивая оперативную память
                    wav_file.writeframes(chunk)
                    total_bytes += len(chunk)

            # Вывод прогресса
            elapsed_time = time.time() - start_time
            mb_size = total_bytes / (1024 * 1024)
            sys.stdout.write(f"\rВремя: {elapsed_time:.1f} сек | Размер файла: {mb_size:.2f} MB")
            sys.stdout.flush()

            # Небольшая пауза, чтобы разгрузить процессор
            time.sleep(0.01)

    except Exception as e:
        print(f"\nПроизошла ошибка во время записи: {e}")
    finally:
        # Корректно закрываем все ресурсы
        ser.close()
        wav_file.close()
        print("\n\n=== ЗАПИСЬ УСПЕШНО ЗАВЕРШЕНА ===")
        print(f"Файл сохранен: {OUTPUT_FILE}")
        print(f"Итоговая длительность: {total_bytes / (SAMPLE_RATE * 2):.1f} сек.")


if __name__ == '__main__':
    main()
