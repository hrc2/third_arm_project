# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'hrc2d_gui.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
# Edited to make reconfigurable
#

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
    def setupUi(self, MainWindow, numbertargets):
        # Create Window
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        self.MainWindowHeight = 800
        self.MainWindowWidth = 800
        self.HistogramBarWidth = 50
        MainWindow.resize(self.MainWindowWidth, self.MainWindowHeight)


        # Create Widget
        self.CentralWidget = QtGui.QWidget(MainWindow)
        self.CentralWidget.setObjectName(_fromUtf8("CentralWidget"))


        # Create Frames
        self.TopFrame = QtGui.QFrame(self.CentralWidget)
        self.BottomFrame = QtGui.QFrame(self.CentralWidget)
        self.TopFrameHeight = self.MainWindowHeight/3
        self.TopFrameWidth = self.MainWindowWidth * 2 / 3
        self.BottomFrameHeight = self.TopFrameHeight
        self.BottomFrameWidth = self.TopFrameWidth
        self.TopFrame.setGeometry(QtCore.QRect(self.MainWindowWidth/2 - self.TopFrameWidth/2, self.MainWindowHeight/4-self.TopFrameHeight/2, self.TopFrameWidth, self.TopFrameHeight))
        self.TopFrame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.TopFrame.setFrameShadow(QtGui.QFrame.Raised)
        self.TopFrame.setObjectName(_fromUtf8("frame"))
        self.BottomFrame.setGeometry(QtCore.QRect(self.MainWindowWidth/2 - self.BottomFrameWidth/2, 3*self.MainWindowHeight/4-self.BottomFrameHeight/2, self.BottomFrameWidth, self.BottomFrameHeight))
        self.BottomFrame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.BottomFrame.setFrameShadow(QtGui.QFrame.Raised)
        self.BottomFrame.setObjectName(_fromUtf8("BottomFrame"))
        self.line = QtGui.QFrame(self.CentralWidget)
        self.line.setGeometry(QtCore.QRect(0, self.MainWindowHeight/2, self.MainWindowWidth, 10))
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))

        # Target Probability Histogram
        self.NumberTargets = numbertargets
        self.ProgressBarTarget = []
        self.LabelsTarget = []
        ProgressBarTop = 0
        ProgressBarLeft = self.TopFrameWidth/(2*self.NumberTargets) - 25
        for i in range(0, self.NumberTargets):
            self.ProgressBarTarget.append(QtGui.QProgressBar(self.TopFrame))
            self.ProgressBarTarget[i].setOrientation(QtCore.Qt.Vertical)
            self.ProgressBarTarget[i].setGeometry(QtCore.QRect(ProgressBarLeft + i*self.TopFrameWidth/self.NumberTargets, ProgressBarTop, self.HistogramBarWidth, self.TopFrameHeight))
            self.ProgressBarTarget[i].setProperty("value", 50)
            self.ProgressBarTarget[i].setObjectName(_fromUtf8("ProgressBar"+str(i)))
            self.LabelsTarget.append(QtGui.QLabel(self.CentralWidget))
            self.LabelsTarget[i].setGeometry(QtCore.QRect((self.MainWindowWidth-self.TopFrameWidth)/2+ProgressBarLeft+i*self.TopFrameWidth/self.NumberTargets-self.HistogramBarWidth/2, self.MainWindowHeight/2 - self.HistogramBarWidth, 2*self.HistogramBarWidth, self.MainWindowHeight/40))
            self.LabelsTarget[i].setObjectName(_fromUtf8("TargetLabel"+str(i)))
            self.LabelsTarget[i].setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)


        # Target Probability Labels
        self.LabelProbabilityTop = QtGui.QLabel(self.CentralWidget)
        self.LabelProbabilityTop.setGeometry(QtCore.QRect(self.MainWindowWidth/40, self.MainWindowHeight/4, self.HistogramBarWidth*2, self.HistogramBarWidth/2))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.LabelProbabilityTop.setFont(font)
        self.LabelProbabilityTop.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.LabelProbabilityTop.setObjectName(_fromUtf8("LabelProbabilityTop"))
        
        self.LabelTopHeading = QtGui.QLabel(self.CentralWidget)
        self.LabelNumberTargets = QtGui.QLabel(self.CentralWidget)
        self.LabelTopHeading.setGeometry(QtCore.QRect(self.MainWindowWidth/3, self.MainWindowHeight/100, self.MainWindowWidth/3, self.MainWindowHeight/20))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.LabelTopHeading.setFont(font)
        self.LabelTopHeading.setObjectName(_fromUtf8("LabelTopHeading"))
        self.LabelNumberTargets.setGeometry(QtCore.QRect(self.MainWindowWidth*0.6, self.MainWindowHeight/100, self.MainWindowWidth/20, self.MainWindowWidth/20))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.LabelNumberTargets.setFont(font)
        self.LabelNumberTargets.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.LabelNumberTargets.setAlignment(QtCore.Qt.AlignCenter)
        self.LabelNumberTargets.setObjectName(_fromUtf8("LabelNumberTargets"))



        # Task State Histogram
        self.ProgressBarTask = []
        self.LabelsTask = []
        ProgressBarTop = 0
        ProgressBarLeft = self.BottomFrameWidth/(2*5) - 25

        for i in range(0, 6):
            self.ProgressBarTask.append(QtGui.QProgressBar(self.BottomFrame))
            self.ProgressBarTask[i].setOrientation(QtCore.Qt.Vertical)
            self.ProgressBarTask[i].setGeometry(QtCore.QRect(ProgressBarLeft + i*self.BottomFrameWidth/5, ProgressBarTop, self.HistogramBarWidth, self.BottomFrameHeight))
            self.ProgressBarTask[i].setProperty("value", 20)
            self.ProgressBarTask[i].setObjectName(_fromUtf8("ProgressBarTask+str(i)"))
            self.LabelsTask.append(QtGui.QLabel(self.CentralWidget))
            self.LabelsTask[i].setGeometry(QtCore.QRect((self.MainWindowWidth-self.BottomFrameWidth)/2+ProgressBarLeft+i*self.BottomFrameWidth/5, self.MainWindowHeight - self.HistogramBarWidth, self.HistogramBarWidth, self.MainWindowHeight / 40))
            self.LabelsTask[i].setObjectName(_fromUtf8("TaskLabel" + str(i)))
            self.LabelsTask[i].setAlignment(QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)

        # Task State Labels
        self.LabelProbabilityBottom = QtGui.QLabel(self.CentralWidget)
        self.LabelProbabilityBottom.setGeometry(QtCore.QRect(self.MainWindowWidth / 40, 3 * self.MainWindowHeight / 4, 2*self.HistogramBarWidth, self.HistogramBarWidth/2))
        font = QtGui.QFont()
        font.setPointSize(11)
        self.LabelProbabilityBottom.setFont(font)
        self.LabelProbabilityBottom.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.LabelProbabilityBottom.setObjectName(_fromUtf8("LabelProbabilityBottom"))

        self.LabelBottomHeading = QtGui.QLabel(self.CentralWidget)
        self.LabelBottomHeading.setGeometry(QtCore.QRect(self.MainWindowWidth/4, self.MainWindowHeight/2 + self.HistogramBarWidth/2, self.MainWindowWidth/2, self.MainWindowHeight/40))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.LabelBottomHeading.setFont(font)
        self.LabelBottomHeading.setAlignment(QtCore.Qt.AlignCenter)
        self.LabelBottomHeading.setObjectName(_fromUtf8("LabelBottomHeading"))
      

        MainWindow.setCentralWidget(self.CentralWidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 800, 30))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow, numbertargets)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow, numbertargets):
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow", None))
        self.LabelTopHeading.setText(_translate("MainWindow", "Number of Targets", None))
        for i in range(0, self.NumberTargets):
            self.LabelsTarget[i].setText(_translate("MainWindow", "Target-"+str(i+1), None))

        TaskLabels = ["Go", "Put", "Close", "Open", "Stop"]
        for i in range(0, 5):
            self.LabelsTask[i].setText(_translate("MainWindow", TaskLabels[i], None))

        self.LabelNumberTargets.setText(_translate("MainWindow", str(numbertargets), None))
        self.LabelBottomHeading.setText(_translate("MainWindow", "Task State", None))
        self.LabelProbabilityTop.setText(_translate("MainWindow", "Probability", None))
        self.LabelProbabilityBottom.setText(_translate("MainWindow", "Probability", None))

