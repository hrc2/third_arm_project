#!/usr/bin/env python

# Followed steps from  (https://nikolak.com/pyqt-qt-designer-getting-started/)
# Instructions (Non-Configurable GUI):
# 1. Make changes to hrc2d_gui.ui using QtDesigner4
# 2. In terminal run : pyuic4 hrc2d_gui.ui -o hrc2d_gui.py
# 3. Use this file to interface the gui with ros
# 4. rosrun third_arm hrc2d_gui_main.py

# Instructions (Configurable GUI):
# 1. Make changes to hrc2d_gui_reconfigurable.py
# 2. Use this file to interface the gui with ros
# 3. rosrun third_arm_hrc2d_gui_main.py
import rospy
from PyQt4 import QtGui
from PyQt4.QtCore import pyqtSignal
from std_msgs.msg import Float32MultiArray
import sys
import hrc2d_gui_reconfigurable
import numpy as np


class HRC2dGuiMain(QtGui.QMainWindow, hrc2d_gui_reconfigurable.Ui_MainWindow):
    # Define triggers to update probabilities in ROSTopic callback functions
    TargetProbabilityTrigger = pyqtSignal(tuple)
    TaskStateTrigger = pyqtSignal(tuple)

    def __init__(self):
        super(self.__class__, self).__init__()

        # #########################Fields#####################################
        self.TargetProbabilityTopic = '/target'
        self.NumTargets = 2
        self.TaskStateTopic = '/task'
        self.NumTasks = 5

        self.DummyValue = np.random.uniform(0, 100)
        self.TargetProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumTargets).tolist())
        self.TaskProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumTasks).tolist())        
        
        # ################### Publishers ####################################

        # ################### Subscribers ####################################
        rospy.Subscriber(self.TargetProbabilityTopic,
                         Float32MultiArray,
                         self.target_probability_callback)

        rospy.Subscriber(self.TaskStateTopic,
                         Float32MultiArray,
                         self.task_state_callback)        

        # ################### Actions ######################################

        # ################### Services #####################################

        # ################### Start-up Procedure ###########################

        self.setupUi(self, self.NumTargets)  # Load the setupUi function from hrc2d_gui.py file

        # Connect triggers to their corresponding methods
        self.TargetProbabilityTrigger.connect(self.update_target_probability)
        self.TaskStateTrigger.connect(self.update_task_state)

    # ################### Action Clients ###################################
    # def actionClient():

    # ################### Service Clients ##################################
    # def ServiceClient():

    # ################### Callback Functions ###############################
    def target_probability_callback(self, msg):
        self.TargetProbabilityTrigger.emit(msg.data)

    def task_state_callback(self, msg):
        self.TaskStateTrigger.emit(msg.data)

    # ################### Methods ###########################################
    def update_target_probability(self, value):
        if len(value) is not self.NumTargets:
            self.NumTargets = len(value)
            #self.reconstruct_ui(self.NumTargets)
            #rospy.sleep(1.0)

        for i in range(0, self.NumTargets):
            self.ProgressBarTarget[i].setProperty("value", value[i])

    def update_task_state(self, value):        
        for i in range(0, self.NumTasks):
            self.ProgressBarTask[i].setProperty("value", value[i])

    def reconstruct_ui(self, value):
        self.setupUi(self, value)  # Load the setupUi function from hrc2d_gui.py file


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('hrc2d_gui', anonymous=False)

    try:
        app = QtGui.QApplication(sys.argv)
        gui = HRC2dGuiMain()
        gui.show()
        app.exec_()
    except rospy.ROSInterruptException:
        pass
