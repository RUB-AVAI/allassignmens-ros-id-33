from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(300, 300)

        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")

        self.image_frame = QtWidgets.QLabel(self.centralwidget)
        self.image_frame.setObjectName("image_frame")
        self.image_frame.setAlignment(QtCore.Qt.AlignCenter)
        self.verticalLayout.addWidget(self.image_frame)

        self.raw_frame = QtWidgets.QLabel(self.centralwidget)
        self.raw_frame.setObjectName("raw_frame")
        self.raw_frame.setAlignment(QtCore.Qt.AlignCenter)
        self.verticalLayout.addWidget(self.raw_frame)

        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        validator = QtGui.QDoubleValidator()
        self.lineEdit.setValidator(validator)
        self.lineEdit.setObjectName('lineEdit')
        self.verticalLayout.addWidget(self.lineEdit)

        self.updateCameraFreqBtn = QtWidgets.QPushButton(self.centralwidget)
        self.updateCameraFreqBtn.setObjectName("updateCameraFreqBtn")
        self.verticalLayout.addWidget(self.updateCameraFreqBtn)

        self.shutterBtn = QtWidgets.QPushButton(self.centralwidget)
        self.shutterBtn.setObjectName("shutterBtn")
        self.verticalLayout.addWidget(self.shutterBtn)

        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Camera Control Center"))
        self.shutterBtn.setText(_translate("MainWindow", "Shutter"))
        self.updateCameraFreqBtn.setText(_translate("MainWindow", "Update Camera Frequency (Hz)"))
