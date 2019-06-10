#!/usr/bin/env python

#Followed steps from  (https://nikolak.com/pyqt-qt-designer-getting-started/)
#Instructions (Non-Configurable GUI):
#1. Make changes to hrc2d_gui.ui using QtDesigner4
#2. In terminal run : pyuic4 hrc2d_gui.ui -o hrc2d_gui.py
#3. Use this file to interface the gui with ros
#4. rosrun third_arm hrc2d_gui_main.py

#Instructions (Configurable GUI):
#1. Make changes to hrc2d_gui_reconfigurable.py
#2. Use this file to interface the gui with ros
#3. rosrun third_arm_hrc2d_gui_main.py
import rospy
from PyQt4 import QtGui
from PyQt4.QtCore import pyqtSignal
import sys
import hrc2d_gui_reconfigurable
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import time

class hrc2d_gui_main(QtGui.QMainWindow, hrc2d_gui_reconfigurable.Ui_MainWindow):
    # Define triggers to update probabilities in rostopic callback functions
    TargetProbabilityTrigger = pyqtSignal(tuple)
    TaskStateTrigger = pyqtSignal(tuple)

    def __init__(self, parent=None):
        super(self.__class__, self).__init__()

        # #########################Fields#####################################
        self.TargetProbabilityTopic = '/target'
        self.TaskStateTopic = '/task'
        self.NumberTargets = 3
        self.NumTasks = 5

        self.DummyValue = np.random.uniform(0, 100)
        self.TargetProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumberTargets).tolist())
        self.TaskProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumTasks).tolist())        
        
        # ################### Publishers ####################################
        self.TargetProbabilityPublisher = rospy.Publisher('/target', Float32MultiArray, queue_size=1)
        self.TaskProbabilityPublisher = rospy.Publisher('/task', Float32MultiArray, queue_size=1)



        # ################### Subscribers ####################################
        rospy.Subscriber(self.TargetProbabilityTopic,
                         Float32MultiArray,
                         self.TargetProbabilityCallback)

        rospy.Subscriber(self.TaskStateTopic,
                         Float32MultiArray,
                         self.TaskStateCallback)        

        
        
        # ################### Actions ######################################

        # ################### Services ####################################

        self.setupUi(self, self.NumberTargets )  # Load the setupUi function from hrc2d_gui.py file

        # Connect triggers to their corresponding methods
        self.TargetProbabilityTrigger.connect(self.UpdateTargetProbability)
        self.TaskStateTrigger.connect(self.UpdateTaskState)

        time.sleep(1)
        self.TargetProbabilityPublisher.publish(self.TargetProbabilities)
        self.TaskProbabilityPublisher.publish(self.TaskProbabilities)
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
        self.NumberTargets = np.random.randint(2,5)
        self.TargetProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumberTargets).tolist())        
        self.setupUi(self, self.NumberTargets)
        time.sleep(1)
        print(len(self.ProgressBarTarget))
        for i in range(0, self.NumberTargets):
            self.ProgressBarTarget[i].setProperty("value", value[i])
        self.TargetProbabilityPublisher.publish(self.TargetProbabilities)

    def UpdateTaskState(self,value):        
        self.DummyValue = np.random.uniform(0,100)        
        self.TaskProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumTasks).tolist())                
        self.TaskProbabilityPublisher.publish(self.TaskProbabilities)   
        
        for i in range(0, self.NumTasks):
            self.ProgressBarTask[i].setProperty("value", value[i])

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
