from design import Ui_MainWindow
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer
import serial

PORT = "COM6"
BAUD_RATE = 115200

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.init_vars()
        
        # Serial port
        try:
            self.serial_port = serial.Serial(PORT, BAUD_RATE, timeout=1)
        except serial.SerialException as e:
            self.serial_port = None
            print(f"Error opening serial port: {e}")
        self.destroyed.connect(self.cleanup)

        self.pushButton.clicked.connect(self.clickHandler)
        self.pushButton_2.clicked.connect(self.clickHandler)
        for button in self.radioButtons: 
            button.clicked.connect(self.radioHandler)
        self.horizontalSlider.valueChanged.connect(self.sliderHandler)
        self.horizontalSlider_2.valueChanged.connect(self.sliderHandler)
    
    def sliderHandler(self, value):
        sliderName = self.sender().objectName()
        if sliderName == 'horizontalSlider':
            self.lcdNumber.display(value)
        elif sliderName == 'horizontalSlider_2':
            self.lcdNumber_2.display(value)
        
    def setDisplay(self, text):
        self.scene.clear()
        self.scene.addText(text)
        self.view = self.graphicsView.setScene(self.scene)

    def radioHandler(self): 
        buttonName = self.sender().objectName()
        self.setDisplay(buttonName)
        print(f'{buttonName} clicked!')
    
    def init_vars(self): 
        self.lcdNumber.display('- - - -')
        self.lcdNumber_2.display('- - - -')

    def clickHandler(self):
        buttonName = self.sender().objectName()
        self.setDisplay(buttonName)
        print(f'{buttonName} clicked!')

    def send_message(self, message):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(f"{message}\n".encode('utf-8'))  
                print(f"Sent to Pico: {message}")
            except serial.SerialException as e:
                print(f"Error sending message: {e}")

    def cleanup(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed.")


app = QApplication([])
window = Window()

window.show()
app.exec()
