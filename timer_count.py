import sys
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
from PyQt5.QtGui import QFont, QPixmap


class Stopwatch(QWidget):
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
        self.lcd.setMinimumSize(400, 150)
        font = self.lcd.font()
        font.setPointSize(36)
        self.lcd.setFont(font)

        self.start_button = QPushButton("Start", self)
        self.pause_button = QPushButton("Pause", self)
        self.reset_button = QPushButton("Reset", self)
        self.timer = QTimer(self)
        self.blink_timer = QTimer(self)
        self.time = QTime(0, 5, 0)  # Countdown starts at 5 minutes
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
        hbox.addWidget(self.pause_button)
        hbox.addWidget(self.reset_button)

        vbox.addLayout(hbox)
        self.setLayout(vbox)

        self.start_button.clicked.connect(self.start)
        self.pause_button.clicked.connect(self.pause)
        self.reset_button.clicked.connect(self.reset)
        self.timer.timeout.connect(self.update_display)
        self.blink_timer.timeout.connect(self.blink_lcd)

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
        """
        )

    def resizeEvent(self, event):
        window_width = self.width()
        image_size1 = int(window_width * 0.18)
        image_size2 = int(window_width * 0.15)

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
        if not self.isStart or self.isPaused:
            self.timer.start(10)
            self.isStart = True
            self.isPaused = False

    def pause(self):
        if not self.isPaused:
            self.timer.stop()
            self.isPaused = True
            self.pause_button.setText("Pause")

    def update_display(self):
        if self.time > self.time_limit:
            self.time = self.time.addMSecs(-10)
            self.lcd.display(self.format_time(self.time))

            if self.time > self.time_range1:
                self.lcd.setStyleSheet("background-color: hsl(39, 100%, 50%);")
            elif self.time > self.time_range2:
                self.lcd.setStyleSheet("background-color: hsl(141, 100%, 50%);")
            else:
                self.lcd.setStyleSheet("background-color: hsl(0, 100%, 60%);")

        else:
            self.timer.stop()
            self.lcd.display("00:00:00")
            self.blink_timer.stop()
            self.reset_lcd_style()

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
        self.lcd.display(self.format_time(self.time))
        self.isStart = False
        self.isPaused = False
        self.pause_button.setText("Pause")
        self.reset_lcd_style()

    def format_time(self, time):
        return f"{time.minute():02}:{time.second():02}:{time.msec() // 10:02}"


if __name__ == "__main__":
    app = QApplication(sys.argv)
    stopwatch = Stopwatch()
    stopwatch.show()
    sys.exit(app.exec_())
