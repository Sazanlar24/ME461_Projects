from design import Ui_MainWindow
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import QApplication, QMainWindow

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.init_vars()

        self.pushButton.clicked.connect(self.textHandler)
        for i in range(1,len(self.buttons)+1): self.buttons[i].clicked.connect(self.clickHandler)
        for button in self.radioButtons: button.clicked.connect(self.radioHandler)
            
    def setDisplay(self, textOrImage:str, textOrPath:str):
        self.scene.clear()
        if textOrImage == 'text': 
            self.scene.addText(textOrPath)
        elif textOrImage == 'image': 
            pixmap = QPixmap(textOrPath)
            self.scene.addPixmap(pixmap)
        else: print('Just provide str(text) or str(image) as first parameter.')

        self.view = self.graphicsView.setScene(self.scene)

    def radioHandler(self): 
        button = self.sender().objectName()
        print(f'{button} clicked!')

    def clickHandler(self):
        self.buttonColor(self.sender(),'green')
        buttonName = self.sender().objectName()
        print(f'{buttonName} clicked!')

    def textHandler(self):
        text = self.textEdit.toPlainText()
        print(f'Text sent: {text}')
        # Assign cell values to map for visual purposes
        if text[:4] == 'path': 
            path = text[5:]
            cellValues = open(path, "r").read().split(',')
            cellValues = [x.rstrip() for x in cellValues]
            for i in range(1,len(self.buttons)+1): self.buttons[i].setText(cellValues[i-1])
                        
    def buttonColor(self, buttonObject:object, color:str):
        buttonObject.setStyleSheet(f"background-color : {color}") 

    def init_vars(self): 
        self.progressBar.setValue(0)
        for button in self.radioButtons: button.setChecked(False)
        self.setDisplay('image', 'home.jpg')

app = QApplication([])
window = Window()

window.show()
app.exec()