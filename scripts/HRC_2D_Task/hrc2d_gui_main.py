#!/usr/bin/env python

#Followed steps from  (https://nikolak.com/pyqt-qt-designer-getting-started/)
#Instructions:
#1. Make changes to hrc2d_gui.ui using QtDesigner4
#2. In terminal run : pyuic4 hrc2d_gui.ui -o hrc2d_gui.py
#3. Use this file to interface the gui with ros
#4. rosrun third_arm hrc2d_gui_main.py

import rospy

from PyQt4 import QtGui
from PyQt4.QtCore import pyqtSignal
import sys
import hrc2d_gui
from std_msgs.msg import String

class hrc2d_gui_main(QtGui.QMainWindow, hrc2d_gui.Ui_MainWindow):
    # Define triggers to update probabilities in rostopic callback functions
    TargetProbabilityTrigger = pyqtSignal(str)
    TaskStateTrigger = pyqtSignal(str)

    def __init__(self, parent=None):
        super(self.__class__, self).__init__()
        self.setupUi(self) #Load the setupUi function from hrc2d_gui.py file

        # Connect triggers to their corresponding methods
        self.TargetProbabilityTrigger.connect(self.UpdateTargetProbability)
        self.TaskStateTrigger.connect(self.UpdateTaskState)

        # #########################Fields#####################################
        self.TargetProbabilityTopic = '/target'
        self.TaskStateTopic = '/task'

        # ################### Subscribers ####################################
        rospy.Subscriber(self.TargetProbabilityTopic,
                         String,
                         self.TargetProbabilityCallback)

        rospy.Subscriber(self.TaskStateTopic,
                         String,
                         self.TaskStateCallback)
        # ################### Publishers ####################################

        # ################### Actions ######################################

        # ################### Services ####################################

    # ################### Action Clients ###################################
    #def actionClient():

    # ################### Service Clients ##################################
    #def ServiceClient():

    # ################### Callback Functions ###############################
    def TargetProbabilityCallback(self,msg):
        self.TargetProbabilityTrigger.emit(msg.data)

    def TaskStateCallback(self,msg):
        self.TaskStateTrigger.emit(msg.data)

    # ################### Methods ###########################################
    def UpdateTargetProbability(self,value):
        self.progressBar.setProperty("value", value)
        self.progressBar_2.setProperty("value", 100-int(value))


    def UpdateTaskState(self,value):
        self.progressBar_3.setProperty("value", value)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('hrc2d_gui', anonymous=False)

    try:
        app = QtGui.QApplication(sys.argv)
        gui = hrc2d_gui_main()
        gui.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass
