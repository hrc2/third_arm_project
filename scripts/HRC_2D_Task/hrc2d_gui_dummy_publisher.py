#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray


class HRC2GuiDummyPublisher:
    def __init__(self):
        # #########################Fields#####################################
        self.DummyValue = np.random.uniform(0, 100)
        self.NumTargets = np.random.randint(1, 5)
        self.NumTasks = 5
        self.TargetProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumTargets).tolist())
        self.TaskProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumTasks).tolist())

        # ################### Publishers ####################################
        self.target_probability_publisher = rospy.Publisher('/target', Float32MultiArray, queue_size=1)
        self.task_probability_publisher = rospy.Publisher('/task', Float32MultiArray, queue_size=1)

    def publish_target_probabilities(self):
        self.DummyValue = np.random.uniform(0, 100)
        self.NumTargets = np.random.randint(1, 5)
        self.TargetProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumTargets).tolist())
        self.target_probability_publisher.publish(self.TargetProbabilities)

    def publish_task_probabilities(self):
        self.DummyValue = np.random.uniform(0, 100)
        self.TaskProbabilities = Float32MultiArray(data=np.repeat(self.DummyValue, self.NumTasks).tolist())
        self.task_probability_publisher.publish(self.TaskProbabilities)


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('hrc2d_gui_dummy_publisher', anonymous=False)

    try:
        DummyPublisher = HRC2GuiDummyPublisher()

        for i in range(1, 10):
            DummyPublisher.publish_target_probabilities()
            rospy.sleep(2)

    except rospy.ROSInterruptException:
        pass




