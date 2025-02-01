import sys
import re

import bluetooth
import threading
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLCDNumber,
    QLabel,
)
from PyQt5.QtCore import QTimer, QTime, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPixmap

ESP32_MACS = [
    "10:06:1C:42:0F:12",  # Replace with actual MAC addresses
    "10:06:1C:82:44:7A",
    "D0:EF:76:48:60:72",
]


class Stopwatch(QWidget):
    stop_signal = pyqtSignal(int)

    def __init__(self):
        super().__init__()

        # Image Labels
        self.image1 = QLabel(self)
        self.image2 = QLabel(self)

        # Load images
        self.pixmap1 = QPixmap("img/eng.png")
        self.pixmap2 = QPixmap("img/mahout.png")

        self.lcd = QLCDNumber(self)
        self.lcd.setDigitCount(8)
        self.lcd.display("05:00:00")
        self.lcd.setMinimumSize(800, 600)
        font = self.lcd.font()
        font.setPointSize(36)
        self.lcd.setFont(font)

        self.start_button = QPushButton("Start", self)
        # self.stop_button1 = QPushButton("Stop1", self)
        # self.stop_button2 = QPushButton("Stop2", self)
        # self.stop_button3 = QPushButton("Stop3", self)
        self.reset_button = QPushButton("Reset", self)

        self.lap_display = [QLabel(), QLabel(), QLabel()]
        self.lap_A = {
            "Team": "A",
            "Time_Stop": False,
            "Time_Out": False,
            "Time": QTime(0, 0, 0),
            "Order": 1,
        }
        self.lap_B = {
            "Team": "B",
            "Time_Stop": False,
            "Time_Out": False,
            "Time": QTime(0, 0, 0),
            "Order": 2,
        }
        self.lap_C = {
            "Team": "C",
            "Time_Stop": False,
            "Time_Out": False,
            "Time": QTime(0, 0, 0),
            "Order": 3,
        }
        self.laps = [self.lap_A, self.lap_B, self.lap_C]

        self.timer = QTimer(self)
        self.blink_timer = QTimer(self)
        self.time = QTime(0, 5, 0)  # Countdown starts at 5 minutes
        self.time_mid = QTime(0, 0, 0)
        self.time_range1 = QTime(0, 4, 0)
        self.time_range2 = QTime(0, 2, 0)
        self.time_limit = QTime(0, 0, 0)
        self.time_warning = QTime(0, 0, 5)
        self.isStart = False
        self.isPaused = False
        self.blinking = False

        self.initUI()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()

    def initUI(self):
        self.setWindowTitle("Countdown Timer")
        # self.showFullScreen()

        vbox = QVBoxLayout()

        # Image layout
        image_layout = QHBoxLayout()
        image_layout.addWidget(self.image1)
        image_layout.addWidget(self.image2)

        vbox.addLayout(image_layout)
        vbox.addWidget(self.lcd)

        hbox = QHBoxLayout()
        hbox.addWidget(self.start_button)
        # hbox.addWidget(self.stop_button1)
        # hbox.addWidget(self.stop_button2)
        # hbox.addWidget(self.stop_button3)
        hbox.addWidget(self.reset_button)

        vbox.addLayout(hbox)
        lap_layout = QHBoxLayout()  # Create a horizontal layout for lap labels
        for index, value in enumerate(self.lap_display):
            value.setProperty("lapse", True)
            value.setAlignment(Qt.AlignCenter)
            team = self.laps[index]["Team"]
            time = self.laps[index]["Time"]
            value.setText(f"LAP {team} {self.format_time(time)}")
            lap_layout.addWidget(value)

        vbox.addLayout(lap_layout)  # Add the horizontal layout to the main layout

        self.setLayout(vbox)

        self.start_button.clicked.connect(self.start)
        # self.stop_button1.clicked.connect(self.stop1)
        # self.stop_button2.clicked.connect(self.stop2)
        # self.stop_button3.clicked.connect(self.stop3)
        self.reset_button.clicked.connect(self.reset)
        self.timer.timeout.connect(self.update_display)
        self.blink_timer.timeout.connect(self.blink_lcd)

        self.stop_signal.connect(self.stop)

        self.setStyleSheet(
            """
            QWidget { background-color: white; }
            QPushButton {
                padding: 20px;
                font-weight: bold;
                font-family: calibri;
                font-size: 25px;
            }
            QLCDNumber {
                font-size: 80px;
                background-color: hsl(39, 100%, 50%);
                border-radius: 20px;
            }
            QLabel[lapse="true"] {
                font-size: 40px;
                background-color: hsl(113, 0%, 69%);
                border-radius: 5px;
            }
        """
        )

    def resizeEvent(self, event):
        window_width = self.width()
        image_size1 = int(window_width * 0.15)
        image_size2 = int(window_width * 0.10)

        self.image1.setPixmap(
            self.pixmap1.scaled(
                image_size1, image_size1, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )
        self.image2.setPixmap(
            self.pixmap2.scaled(
                image_size2, image_size2, Qt.KeepAspectRatio, Qt.SmoothTransformation
            )
        )

        self.image1.setFixedSize(image_size1, image_size1)
        self.image2.setFixedSize(image_size2, image_size2)

    def start(self):
        self.timer.start(10)

    def stop(self, data):
        # print(data)
        for value in self.laps:
            if value["Order"] == data and self.isStart:
                value["Time_Stop"] = True

    # def stop2(self):
    #     for value in self.laps:
    #         if value["Order"] == 2 and self.isStart:
    #             value["Time_Stop"] = True

    # def stop3(self):
    #     for value in self.laps:
    #         if value["Order"] == 3 and self.isStart:
    #             value["Time_Stop"] = True

    def update_display(self):
        if self.time > self.time_limit:
            self.time = self.time.addMSecs(-10)
            self.lcd.display(self.format_time(self.time))
            if QTime(0, 1, 0) < self.time <= QTime(0, 4, 0):
                self.time_mid = self.time_mid.addMSecs(10)
                if not self.laps[0]["Time_Stop"]:
                    self.laps[0]["Time"] = self.time_mid
                if not self.laps[1]["Time_Stop"]:
                    self.laps[1]["Time"] = self.time_mid
                if not self.laps[2]["Time_Stop"]:
                    self.laps[2]["Time"] = self.time_mid
                if not self.isStart:
                    self.isStart = True
            elif self.time <= QTime(0, 1, 0):
                for value in self.laps:
                    if not value["Time_Stop"]:
                        value["Time_Out"] = True

            if self.time > QTime(0, 4, 0):
                self.lcd.setStyleSheet("background-color: hsl(39, 100%, 50%);")
            elif self.time > QTime(0, 1, 0):
                self.lcd.setStyleSheet("background-color: hsl(141, 100%, 50%);")
            else:
                self.lcd.setStyleSheet("background-color: hsl(0, 100%, 60%);")

        else:
            self.timer.stop()
            self.lcd.display("00:00:00")
            self.blink_timer.stop()
            self.reset_lcd_style()

        for index, value in enumerate(self.lap_display):
            team = self.laps[index]["Team"]
            time = self.laps[index]["Time"]
            if not self.laps[index]["Time_Out"]:
                value.setText(f"LAP {team} {self.format_time(time)}")
            else:
                value.setText(f"LAP {team} 03:00:00")

    def blink_lcd(self):
        if not self.blinking:
            self.lcd.setStyleSheet("background-color: red;")
        else:
            self.lcd.setStyleSheet("background-color: hsl(0, 100%, 60%);")
        self.blinking = not self.blinking

    def reset_lcd_style(self):
        self.lcd.setStyleSheet("background-color: hsl(39, 100%, 50%);")
        self.blinking = False

    def reset(self):
        self.timer.stop()
        self.blink_timer.stop()
        self.time = QTime(0, 5, 0)
        self.time_mid = QTime(0, 0, 0)
        self.lcd.display(self.format_time(self.time))
        self.isStart = False
        self.isPaused = False
        self.reset_lcd_style()

        for index, lap in enumerate(self.laps):
            lap["Time_Stop"] = False
            lap["Time"] = QTime(0, 0, 0)
            lap["Time_Out"] = False

        self.laps = sorted(self.laps, key=lambda lap: str(lap["Team"]))

        for index, value in enumerate(self.lap_display):
            team = self.laps[index]["Team"]
            time = self.laps[index]["Time"]
            value.setText(f"LAP {team} {self.format_time(time)}")

    def format_time(self, time):
        return f"{time.minute():02}:{time.second():02}:{time.msec() // 10:02}"

    def update_from_bluetooth(self, command):
        print(command)
        if command == "1":
            self.stop_signal.emit(1)
        elif command == "2":
            self.stop_signal.emit(2)
        elif command == "3":
            self.stop_signal.emit(3)

    def connect_status(self, data):
        self.lap_display[data].setStyleSheet("background-color: hsl(185, 86%, 69%);")


class BluetoothThread(QThread):
    def __init__(self, address, stopwatch_instance):
        super().__init__()
        self.mac_address = address  # Store the MAC address
        self.stopwatch = stopwatch_instance

    def run(self):
        self.handle_client()

    def handle_client(self):
        sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        try:
            sock.connect((self.mac_address, 1))  # RFCOMM channel 1
            print(f"Connected to {self.mac_address}")
            self.stopwatch.connect_status(ESP32_MACS.index(self.mac_address))

            while True:
                data = sock.recv(1024)  # Receive data
                numbers = re.findall(r"\d+", data.decode("utf-8"))
                if len(numbers) != 0:
                    self.stopwatch.update_from_bluetooth(numbers[0])
        except Exception as e:
            print(f"Error with {self.mac_address}: {e}")
        finally:
            sock.close()
            print(f"Disconnected from {self.mac_address}")


def main():
    app = QApplication(sys.argv)
    stopwatch = Stopwatch()

    bt_threads = []
    for device in ESP32_MACS:
        bt_thread = BluetoothThread(
            address=device,
            stopwatch_instance=stopwatch,
        )
        bt_threads.append(bt_thread)
        bt_thread.start()

    stopwatch.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
