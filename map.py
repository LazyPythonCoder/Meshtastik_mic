# "904026dd": {"name": "Нода 1", "lat": 55.592860, "lon": 37.612396, "status": "CLEAR"},
# "a7eef537": {"name": "Нода 2", "lat": 55.593166, "lon": 37.614498, "status": "CLEAR"}


import threading
import time
import dash
from dash import dcc, html
from dash.dependencies import Input, Output
import dash_leaflet as dl
import meshtastic
import meshtastic.serial_interface
import winsound

# ==========================================
# 1. НАСТРОЙКА ДАННЫХ И КООРДИНАТ НОД
# ==========================================

# Ваш словарь с фиксированными координатами нод.
# Ключ (ID ноды) должен совпадать с тем, как нода идентифицирует себя в сети Meshtastic (например, её Node ID или имя)
NODES_DATABASE = {
    "!904026dd": {"name": "Нода-Перевал", "lat": 55.7558, "lon": 37.6173},
    "!a7eef537": {"name": "Нода-Долина", "lat": 55.7517, "lon": 37.6011},
    "Node_Three": {"name": "Нода-Лес", "lat": 55.7600, "lon": 37.6300},
}

# Глобальный словарь для хранения текущего статуса безопасности (по умолчанию все CLEAR)
# Структура: { node_id: "CLEAR" или "ALARM" }
node_statuses = {node_id: "CLEAR" for node_id in NODES_DATABASE}

# Потокобезопасный лок для одновременного доступа из Meshtastic и Dash
status_lock = threading.Lock()


# ==========================================
# 2. ОБРАБОТКА ДАННЫХ ИЗ MESHTASTIC (USB)
# ==========================================

def on_receive_message(packet, interface):
    """Callback-функция, которая срабатывает при получении любого пакета."""
    try:
        # Проверяем, что в пакете есть текстовое сообщение (portnum == TEXT_MESSAGE_APP)
        if packet.get("decoded", {}).get("portnum") == "TEXT_MESSAGE_APP":
            sender_id = packet.get("fromId")  # ID отправителя (например, '!248a3c10')
            message_text = packet["decoded"]["text"].strip().upper()

            print(f"[Meshtastic] Получено сообщение от {sender_id}: {message_text}")

            # Если нода есть в нашей базе данных, обрабатываем её статус
            if sender_id in node_statuses:
                if "ALARM" in message_text:
                    with status_lock:
                        node_statuses[sender_id] = "ALARM"
                    print(f" СТАТУС ОБНОВЛЕН: {sender_id} -> ALARM")
                    winsound.Beep(frequency=800, duration=500)
                elif "CLEAR" in message_text:
                    with status_lock:
                        node_statuses[sender_id] = "CLEAR"
                    print(f" СТАТУС ОБНОВЛЕН: {sender_id} -> CLEAR")

    except Exception as e:
        print(f"Ошибка при разборе пакета: {e}")


def start_meshtastic():
    """Инициализация USB-подключения к базовой ноде."""
    print("Подключение к Meshtastic ноде через USB...")
    try:
        # Автоматически находит подключенную по USB ноду.
        # Если портов несколько, можно указать явно, например: devPath='/dev/ttyUSB0' или 'COM3'
        interface = meshtastic.serial_interface.SerialInterface()

        # Подписываемся на событие получения текстовых сообщений
        from pubsub import pub
        pub.subscribe(on_receive_message, "meshtastic.receive.text")

        print("Интерфейс Meshtastic успешно запущен и слушает эфир.")
    except Exception as e:
        print(f"Не удалось подключиться к USB-ноде: {e}")
        print("Программа продолжит работу в режиме симуляции карты.")


# ==========================================
# 3. ИНТЕРАКТИВНАЯ КАРТА (DASH & LEAFLET)
# ==========================================

app = dash.Dash(__name__)

# Стили для кастомных круглых маркеров (вместо стандартных синих капель)
alarm_marker_style = {
    "background-color": "red",
    "border-radius": "50%",
    "border": "2px solid white",
    "width": "20px",
    "height": "20px"
}

clear_marker_style = {
    "background-color": "green",
    "border-radius": "50%",
    "border": "2px solid white",
    "width": "20px",
    "height": "20px"
}

# Макет страницы
app.layout = html.Div([
    html.H1("Мониторинг безопасности нод Meshtastic", style={"textAlign": "center", "fontFamily": "Arial"}),

    # Компонент карты
    dl.Map(
        id="map",
        center=[55.7558, 37.6173],  # Центрирование (по умолчанию Москва)
        zoom=13,
        children=[
            dl.TileLayer(),  # Базовый слой карты (OpenStreetMap)
            html.Div(id="markers-layer")  # Динамический слой, куда мы будем рендерить маркеры
        ],
        style={'width': '100%', 'height': '80vh'}
    ),

    # Интервал обновления карты в миллисекундах (2000 мс = 2 секунды)
    dcc.Interval(
        id='interval-component',
        interval=2000,
        n_intervals=0
    )
])


@app.callback(
    Output("markers-layer", "children"),
    Input("interval-component", "n_intervals")
)
def update_markers(n):
    """Каждые 2 секунды забирает актуальные статусы и перерисовывает круглые маркеры."""
    markers = []

    with status_lock:
        current_statuses = node_statuses.copy()

    for node_id, info in NODES_DATABASE.items():
        status = current_statuses.get(node_id, "CLEAR")

        # Определяем цвет в зависимости от статуса
        color_node = "red" if status == "ALARM" else "green"

        # Используем CircleMarker вместо DivMarker
        marker = dl.CircleMarker(
            center=[info["lat"], info["lon"]],
            radius=10,  # Размер точки
            color="white",  # Цвет обводки круга
            weight=2,  # Толщина обводки в пикселях
            fillColor=color_node,  # Внутренний цвет (Красный или Зеленый)
            fillOpacity=0.9,  # Прозрачность заливки
            children=[
                dl.Popup([
                    html.B(info["name"]),
                    html.Br(),
                    html.Span(f"ID: {node_id}"),
                    html.Br(),
                    html.Span(f"Статус: {status}", style={"color": color_node, "fontWeight": "bold"})
                ])
            ]
        )
        markers.append(marker)

    return markers

# ==========================================
# 4. ЗАПУСК ПРИЛОЖЕНИЯ
# ==========================================

if __name__ == "__main__":
    meshtastic_thread = threading.Thread(target=start_meshtastic, daemon=True)
    meshtastic_thread.start()

    # Новый актуальный метод:
    app.run(debug=False, port=8050)

