# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'hrc2d_gui.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(800, 685)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.label = QtGui.QLabel(self.centralwidget)
        self.label.setGeometry(QtCore.QRect(250, 9, 221, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        self.frame = QtGui.QFrame(self.centralwidget)
        self.frame.setGeometry(QtCore.QRect(200, 60, 391, 201))
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName(_fromUtf8("frame"))
        self.progressBar = QtGui.QProgressBar(self.frame)
        self.progressBar.setGeometry(QtCore.QRect(40, 0, 51, 201))
        self.progressBar.setProperty("value", 50)
        self.progressBar.setOrientation(QtCore.Qt.Vertical)
        self.progressBar.setObjectName(_fromUtf8("progressBar"))
        self.progressBar_2 = QtGui.QProgressBar(self.frame)
        self.progressBar_2.setGeometry(QtCore.QRect(300, 0, 51, 201))
        self.progressBar_2.setProperty("value", 50)
        self.progressBar_2.setOrientation(QtCore.Qt.Vertical)
        self.progressBar_2.setObjectName(_fromUtf8("progressBar_2"))
        self.label_2 = QtGui.QLabel(self.centralwidget)
        self.label_2.setGeometry(QtCore.QRect(230, 270, 83, 22))
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label_3 = QtGui.QLabel(self.centralwidget)
        self.label_3.setGeometry(QtCore.QRect(480, 270, 83, 22))
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.label_4 = QtGui.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(470, 10, 31, 41))
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.line = QtGui.QFrame(self.centralwidget)
        self.line.setGeometry(QtCore.QRect(0, 290, 801, 20))
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.label_5 = QtGui.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(100, 150, 91, 22))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_5.setFont(font)
        self.label_5.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.frame_2 = QtGui.QFrame(self.centralwidget)
        self.frame_2.setGeometry(QtCore.QRect(120, 360, 551, 201))
        self.frame_2.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_2.setObjectName(_fromUtf8("frame_2"))
        self.progressBar_3 = QtGui.QProgressBar(self.frame_2)
        self.progressBar_3.setGeometry(QtCore.QRect(50, 20, 51, 201))
        self.progressBar_3.setProperty("value", 20)
        self.progressBar_3.setOrientation(QtCore.Qt.Vertical)
        self.progressBar_3.setObjectName(_fromUtf8("progressBar_3"))
        self.progressBar_4 = QtGui.QProgressBar(self.frame_2)
        self.progressBar_4.setGeometry(QtCore.QRect(150, 20, 51, 201))
        self.progressBar_4.setProperty("value", 20)
        self.progressBar_4.setOrientation(QtCore.Qt.Vertical)
        self.progressBar_4.setObjectName(_fromUtf8("progressBar_4"))
        self.progressBar_5 = QtGui.QProgressBar(self.frame_2)
        self.progressBar_5.setGeometry(QtCore.QRect(250, 20, 51, 201))
        self.progressBar_5.setProperty("value", 20)
        self.progressBar_5.setOrientation(QtCore.Qt.Vertical)
        self.progressBar_5.setObjectName(_fromUtf8("progressBar_5"))
        self.progressBar_6 = QtGui.QProgressBar(self.frame_2)
        self.progressBar_6.setGeometry(QtCore.QRect(350, 20, 51, 201))
        self.progressBar_6.setProperty("value", 20)
        self.progressBar_6.setOrientation(QtCore.Qt.Vertical)
        self.progressBar_6.setObjectName(_fromUtf8("progressBar_6"))
        self.progressBar_7 = QtGui.QProgressBar(self.frame_2)
        self.progressBar_7.setGeometry(QtCore.QRect(450, 20, 51, 201))
        self.progressBar_7.setProperty("value", 20)
        self.progressBar_7.setOrientation(QtCore.Qt.Vertical)
        self.progressBar_7.setObjectName(_fromUtf8("progressBar_7"))
        self.label_6 = QtGui.QLabel(self.centralwidget)
        self.label_6.setGeometry(QtCore.QRect(320, 320, 151, 22))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label_6.setFont(font)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.label_7 = QtGui.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(20, 450, 91, 22))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.label_7.setFont(font)
        self.label_7.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.label_8 = QtGui.QLabel(self.centralwidget)
        self.label_8.setGeometry(QtCore.QRect(150, 570, 81, 22))
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.label_9 = QtGui.QLabel(self.centralwidget)
        self.label_9.setGeometry(QtCore.QRect(250, 570, 81, 22))
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.label_10 = QtGui.QLabel(self.centralwidget)
        self.label_10.setGeometry(QtCore.QRect(360, 570, 81, 22))
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.label_11 = QtGui.QLabel(self.centralwidget)
        self.label_11.setGeometry(QtCore.QRect(460, 570, 81, 22))
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.label_12 = QtGui.QLabel(self.centralwidget)
        self.label_12.setGeometry(QtCore.QRect(560, 570, 81, 22))
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 30))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.label.setText(_translate("MainWindow", "Number of Targets", None))
        self.label_2.setText(_translate("MainWindow", "Target-1", None))
        self.label_3.setText(_translate("MainWindow", "Target-2", None))
        self.label_4.setText(_translate("MainWindow", "2", None))
        self.label_5.setText(_translate("MainWindow", "Probability", None))
        self.label_6.setText(_translate("MainWindow", "Task State", None))
        self.label_7.setText(_translate("MainWindow", "Probability", None))
        self.label_8.setText(_translate("MainWindow", "Go", None))
        self.label_9.setText(_translate("MainWindow", "Put", None))
        self.label_10.setText(_translate("MainWindow", "Close", None))
        self.label_11.setText(_translate("MainWindow", "Open", None))
        self.label_12.setText(_translate("MainWindow", "Stop", None))
