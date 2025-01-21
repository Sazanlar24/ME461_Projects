from design import Ui_MainWindow
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer
import serial
import time

PORT = "COM6"
BAUD_RATE = 115200

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # Initialize serial port
        try:
            self.serial_port = serial.Serial(PORT, BAUD_RATE, timeout=1)
        except serial.SerialException as e:
            self.serial_port = None
            print(f"Error opening serial port: {e}")

        self.pushButton.clicked.connect(self.clickHandler)
        self.pushButton_2.clicked.connect(self.clickHandler)
        self.pushButton_3.clicked.connect(self.clickHandler)
        
        self.init_vars()

        # Close serial port on exit
        self.destroyed.connect(self.cleanup)

    def init_vars(self): 
        self.order = 0
        self.progressBar.setValue(0)
        self.lcdNumber.display('- - - -')

    def clickHandler(self):
        button = self.sender().objectName()
        delay, seq = self.get_text()
        
        line12Start = find_nth_overlapping(seq, '\n', 12)
        if line12Start != -1: seq = seq[:line12Start]

        if button == 'pushButton':  
            self.send_message('STATE:MANUAL')
            nlines = seq.count('\n')
            self.order  = self.order % (nlines+1)
            coil = seq[self.order*8:self.order*8+8]
            self.lcdNumber.display(coil)
            self.progressBar.setValue(int((self.order+1)*100/(nlines+1)))
            self.send_message('COIL:{}'.format(coil))
            self.order += 1
            
        elif button == 'pushButton_2': 
            self.init_vars() 
            self.send_message('STATE:AUTO')
            self.send_message('DELAY:{}'.format(delay))
            
            msg = ''
            seq = seq.splitlines()
            for i in seq: 
                msg += i
                msg += '-'
            self.send_message('SEQ:{}'.format(msg))

        else: 
            self.send_message('STATE:STOP')
            self.init_vars()

    def get_text(self): 
        delay = self.textEdit.toPlainText()
        try:
            delay = float(delay)
        except: delay = 500
        seq = self.textEdit_2.toPlainText()
        return delay, seq 

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

def find_nth_overlapping(haystack, needle, n):
    start = haystack.find(needle)
    while start >= 0 and n > 1:
        start = haystack.find(needle, start+1)
        n -= 1
    return start

app = QApplication([])
window = Window()

window.show()
app.exec()
