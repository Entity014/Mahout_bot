import sys
import RPi.GPIO as GPIO
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QLCDNumber,
    QLabel,
)
from PyQt5.QtCore import QTimer, QTime, Qt
from PyQt5.QtGui import QFont


class Stopwatch(QWidget):
    def __init__(self):
        super().__init__()
        self.lcd = QLCDNumber(self)
        self.lcd.setDigitCount(8)
        self.lcd.display("00:00:00")
        self.lcd.setMinimumSize(400, 150)
        font = self.lcd.font()
        font.setPointSize(36)
        self.lcd.setFont(font)

        # self.start_button = QPushButton("Start", self)
        # self.stop_button1 = QPushButton("Stop1", self)
        # self.stop_button2 = QPushButton("Stop2", self)
        # self.stop_button3 = QPushButton("Stop3", self)
        # self.reset_button = QPushButton("Reset", self)
        self.timer_button = QTimer(self)
        self.timer = QTimer(self)
        self.blink_timer = QTimer(self)
        self.time = QTime(0, 0, 0)
        self.time_limit = QTime(0, 0, 10)
        self.time_warning = QTime(0, 0, 5)
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
        self.isStart = False
        self.blinking = False

        self.GPIO.setmode(GPIO.BCM)
        self.GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.GPIO.setup(22, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.GPIO.setup(24, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.initUI()

    def initUI(self):
        self.setWindowTitle("Stopwatch")

        vbox = QVBoxLayout()
        vbox.addWidget(self.lcd)
        self.setLayout(vbox)

        self.lcd.setSegmentStyle(QLCDNumber.Flat)

        hbox = QHBoxLayout()

        # hbox.addWidget(self.start_button)
        # hbox.addWidget(self.stop_button1)
        # hbox.addWidget(self.stop_button2)
        # hbox.addWidget(self.stop_button3)
        # hbox.addWidget(self.reset_button)

        vbox.addLayout(hbox)
        for index, value in enumerate(self.lap_display):
            vbox.addWidget(value)
            value.setAlignment(Qt.AlignCenter)
            team = self.laps[index]["Team"]
            time = self.laps[index]["Time"]
            value.setText(f"LAP {team} {self.format_time(time)}")

        self.setStyleSheet(
            """
            QPushButton{
                padding: 20px;
                font-weight: bold;
                font-family: calibri;
                font-size: 25px;
            }
            QLCDNumber{
                font-size: 80px;
                background-color: hsl(200, 100%, 85%);
                border-radius: 20px;
            }
            QLabel{
                font-size: 150px;
                background-color: hsl(200, 100%, 85%);
                border-radius: 20px;
            }
        """
        )

        self.timer_button.start(10)
        # self.start_button.clicked.connect(self.start)
        # self.stop_button1.clicked.connect(self.stop1)
        # self.stop_button2.clicked.connect(self.stop2)
        # self.stop_button3.clicked.connect(self.stop3)
        # self.reset_button.clicked.connect(self.reset)
        self.timer.timeout.connect(self.update_display)
        self.blink_timer.timeout.connect(self.blink_lcd)
        self.timer_button.timeout.connect(self.button_callback)

    def button_callback(self):
        self.GPIO.add_event_detect(23, GPIO.RISING, callback=self.start, bouncetime=300)
        self.GPIO.add_event_detect(17, GPIO.RISING, callback=self.stop1, bouncetime=300)
        self.GPIO.add_event_detect(27, GPIO.RISING, callback=self.stop2, bouncetime=300)
        self.GPIO.add_event_detect(22, GPIO.RISING, callback=self.stop3, bouncetime=300)
        self.GPIO.add_event_detect(24, GPIO.RISING, callback=self.reset, bouncetime=300)

    def start(self):
        self.timer.start(10)
        self.isStart = True

    def stop1(self):
        for value in self.laps:
            if value["Order"] == 1 and self.isStart:
                value["Time_Stop"] = True

    def stop2(self):
        for value in self.laps:
            if value["Order"] == 2 and self.isStart:
                value["Time_Stop"] = True

    def stop3(self):
        for value in self.laps:
            if value["Order"] == 3 and self.isStart:
                value["Time_Stop"] = True

    def format_time(self, time):
        minutes = time.minute()
        seconds = time.second()
        milliseconds = time.msec() // 10
        return f"{minutes:02}:{seconds:02}:{milliseconds:02}"

    def update_display(self):
        self.lcd.display(self.format_time(self.time))
        if not (self.time >= self.time_limit):
            if not self.laps[0]["Time_Stop"]:
                self.laps[0]["Time"] = self.laps[0]["Time"].addMSecs(10)
            if not self.laps[1]["Time_Stop"]:
                self.laps[1]["Time"] = self.laps[1]["Time"].addMSecs(10)
            if not self.laps[2]["Time_Stop"]:
                self.laps[2]["Time"] = self.laps[2]["Time"].addMSecs(10)
            if (
                not self.laps[0]["Time_Stop"]
                or not self.laps[1]["Time_Stop"]
                or not self.laps[2]["Time_Stop"]
            ):
                self.time = self.time.addMSecs(10)
        else:
            for value in self.laps:
                if not value["Time_Stop"]:
                    value["Time_Out"] = True

        if (
            self.time >= self.time_warning
            and self.time < self.time_limit
            and (
                not self.laps[0]["Time_Stop"]
                or not self.laps[1]["Time_Stop"]
                or not self.laps[2]["Time_Stop"]
            )
        ):
            if not self.blink_timer.isActive():
                self.blink_timer.start(300)
        else:
            self.blink_timer.stop()
            self.reset_lcd_style()

        self.laps = sorted(
            self.laps, key=lambda lap: lap["Time"].msecsSinceStartOfDay()
        )

        for index, value in enumerate(self.lap_display):
            team = self.laps[index]["Team"]
            time = self.laps[index]["Time"]
            if not self.laps[index]["Time_Out"]:
                value.setText(f"LAP {team} {self.format_time(time)}")
            else:
                value.setText(f"LAP {team} time out")

    def blink_lcd(self):
        if not self.blinking:
            self.lcd.setStyleSheet("background-color: red;")
        else:
            self.lcd.setStyleSheet("background-color: hsl(200, 100%, 85%);")
        self.blinking = not self.blinking

    def reset_lcd_style(self):
        self.lcd.setStyleSheet("background-color: hsl(200, 100%, 85%);")
        self.blinking = False

    def reset(self):
        self.timer.stop()
        self.time = QTime(0, 0, 0)
        self.lcd.display("00:00:00")
        self.isStart = False
        self.blink_timer.stop()
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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    stopwatch = Stopwatch()
    stopwatch.show()
    sys.exit(app.exec_())
