import threading
import json
from flask import Flask, jsonify
import folium
from pubsub import pub
import meshtastic
import meshtastic.serial_interface

app = Flask(__name__)

# Известные координаты нод
NODE_COORDINATES = {
    "904026dd": [55.592860, 37.612396]
}

# Текущие статусы нод (по умолчанию зеленый)
node_statuses = {
    "904026dd": "green"
}


# 1. Функция обработки входящих сообщений от Meshtastic
def on_receive_packet(packet, interface):
    try:
        if 'decoded' in packet and packet['decoded'].get('portnum') == 'TEXT_MESSAGE_APP':
            from_id_raw = packet.get('fromId', '')
            node_id = from_id_raw.replace('!', '').lower()

            if node_id in NODE_COORDINATES:
                message = packet['decoded']['payload'].decode('utf-8').strip()
                print(f"Получено сообщение от {node_id}: {message}")

                if message == "ALARM":
                    node_statuses[node_id] = "red"
                elif message == "CLEAR":
                    node_statuses[node_id] = "green"
    except Exception as e:
        print(f"Ошибка при обработке пакета: {e}")


# 2. Инициализация Meshtastic в отдельном потоке
def start_meshtastic():
    print("Подключение к ноде Meshtastic через USB...")
    try:
        pub.subscribe(on_receive_packet, "meshtastic.receive")
        # Автоматически находит подключенную по USB ноду
        interface = meshtastic.serial_interface.SerialInterface()
    except Exception as e:
        print(f"Не удалось подключиться к USB-ноде: {e}. Работа в демо-режиме.")


# 3. Маршрут для получения статуса (API для фронтенда)
@app.route('/api/status')
def get_status():
    return jsonify(node_statuses)


# 4. Главная страница с картой
@app.route('/')
def index():
    start_coords = NODE_COORDINATES["904026dd"]

    # Создаем карту Folium
    m = folium.Map(location=start_coords, zoom_start=14)

    # Создаем маркер средствами Folium. Он сразу будет на карте при загрузке.
    # Добавляем кастомный класс 'node-marker-904026dd', чтобы легко найти его через JS.
    marker = folium.Marker(
        location=start_coords,
        popup="<b>Нода:</b> 904026dd",
        icon=folium.Icon(color=node_statuses["904026dd"], icon="info-sign")
    )
    marker.add_to(m)

    # Кастомный JS, который находит существующий маркер по его HTML-классу
    # и меняет иконку (AwesomeMarkers) на лету без перезагрузки всей карты.
    custom_js = """
    <script>
    document.addEventListener("DOMContentLoaded", function() {
        // Карта Folium генерирует маркеры с классами. Мы найдем ID внутреннего Leaflet-объекта маркера.
        setTimeout(function() {
            // Функция опроса статуса
            function updateMarkers() {
                fetch('/api/status')
                    .then(response => response.json())
                    .then(statuses => {
                        for (var node_id in statuses) {
                            var color = statuses[node_id]; // "red" или "green"

                            // Находим элемент маркера на карте по уникальному стилю или классу Leaflet
                            // Folium по умолчанию оборачивает маркеры в стандартные иконки Leaflet
                            // Проще всего найти элемент с кластом awesome-marker-icon-ЦВЕТ и обновить его класс
                            var markerElements = document.querySelectorAll('.awesome-marker-icon-green, .awesome-marker-icon-red');

                            markerElements.forEach(function(el) {
                                // Если статус изменился, подменяем CSS-классы иконки, чтобы поменять цвет
                                if (color === 'red' && el.classList.contains('awesome-marker-icon-green')) {
                                    el.classList.remove('awesome-marker-icon-green');
                                    el.classList.add('awesome-marker-icon-red');
                                } else if (color === 'green' && el.classList.contains('awesome-marker-icon-red')) {
                                    el.classList.remove('awesome-marker-icon-red');
                                    el.classList.add('awesome-marker-icon-green');
                                }
                            });
                        }
                    })
                    .catch(err => console.error("Ошибка обновления статуса:", err));
            }

            // Запускаем интервал опроса каждые 2 секунды
            setInterval(updateMarkers, 2000);
        }, 1000);
    });
    </script>
    """

    # Внедряем скрипт в карту
    m.get_root().html.add_child(folium.Element(custom_js))

    return m.get_root().render()


if __name__ == '__main__':
    mesh_thread = threading.Thread(target=start_meshtastic, daemon=True)
    mesh_thread.start()

    app.run(debug=True, use_reloader=False)
