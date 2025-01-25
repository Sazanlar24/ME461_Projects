from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
import serial
import time

PORT = "COM7"
BAUD_RATE = 115200

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(428, 282)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.verticalLayoutWidget = QWidget(self.centralwidget)
        self.verticalLayoutWidget.setObjectName(u"verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(170, 60, 251, 112))
        self.verticalLayout = QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.button_release = QPushButton(self.verticalLayoutWidget)
        self.button_release.setObjectName(u"button_release")
        font = QFont()
        font.setPointSize(16)
        self.button_release.setFont(font)
        self.verticalLayout.addWidget(self.button_release)

        self.label_custom_pos = QLabel(self.verticalLayoutWidget)
        self.label_custom_pos.setObjectName(u"label_custom_pos")
        self.label_custom_pos.setFont(font)
        self.label_custom_pos.setAlignment(Qt.AlignCenter)
        self.verticalLayout.addWidget(self.label_custom_pos)

        self.scroll = QScrollBar(self.verticalLayoutWidget)
        self.scroll.setOrientation(Qt.Horizontal)
        self.verticalLayout.addWidget(self.scroll)

        self.verticalLayoutWidget_2 = QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QRect(10, 10, 158, 215))
        self.verticalLayout_2 = QVBoxLayout(self.verticalLayoutWidget_2)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.radio_buttons = []
        for angle in [0, 45, 90, 135, 180]:
            button = QRadioButton(self.verticalLayoutWidget_2)
            button.setText(str(angle))
            button.setFont(font)
            button.toggled.connect(lambda checked, a=angle: self.button_clicked(a) if checked else None)
            self.verticalLayout_2.addWidget(button)
            self.radio_buttons.append(button)

        MainWindow.setCentralWidget(self.centralwidget)
        MainWindow.setMenuBar(QMenuBar(MainWindow))
        MainWindow.setStatusBar(QStatusBar(MainWindow))

        # Connect signals
        self.scroll.valueChanged.connect(self.print_scroll_value)
        self.button_release.clicked.connect(self.release_command)

        # Initialize serial port
        try:
            self.serial_port = serial.Serial(PORT, BAUD_RATE, timeout=1)
        except serial.SerialException as e:
            self.serial_port = None
            print(f"Error opening serial port: {e}")

        self.retranslateUi(MainWindow)

        # Close serial port on exit
        MainWindow.destroyed.connect(self.cleanup)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle("MainWindow")
        self.button_release.setText("RELEASE")
        self.label_custom_pos.setText("Number")

    def button_clicked(self, number):
        self.degree = number
        self.scroll.setValue(number * 100 // 180)
        #self.send_message(f"SET {number}")

    def print_scroll_value(self):
        self.degree = self.scroll.value() * 180 // 100
        if self.degree == 178:
            self.degree = 180
        self.send_message(f"{self.degree}")
        self.label_custom_pos.setText(str(self.degree))

    def release_command(self):
        self.send_message("RELEASE")

    def send_message(self, message):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{message}\n".encode('utf-8'))
                #print(f"Sent to Pico: {message}")
            except serial.SerialException as e:
                print(f"Error sending message: {e}")

    def cleanup(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")

if __name__ == "_main_":
    import sys
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
