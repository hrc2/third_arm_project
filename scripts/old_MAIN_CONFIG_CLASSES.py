motors = {
    1: 'MX_64', #https://emanual.robotis.com/docs/en/dxl/mx/mx-64/
    2: 'MX_64', # ^
    3: 'MX_28', #https://emanual.robotis.com/docs/en/dxl/mx/mx-28/
    4: 'AX_12A', #https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
    5: 'AX_12A', # ^
    6: 'MX_28' # ^^^
}

class WRTA_ROS_config:

    def __init__(self):
        ## TOPICS ##
        self.trajectory_topic_name = '/dynamixel_workbench/joint_trajectory'

        ## SERVICES ##

        self.command_service = "/dynamixel_workbench/dynamixel_command"
        self.execution_service = "/dynamixel_workbench/execution"


class MainMotor:


    def __init__(self, service_caller, config_class):
        """instantiate the service_caller for controlling this motor
        Args:
            service_caller: function that will communicate to dynamixal command
            motor_number: the number for this motor
            configClass: config for this motor
        """

        self.Goal_Position = 'Goal_Position'
        self.Moving_Speed = 'Moving_Speed'
        self.Present_Position = 'Present_Position'
        self.Present_Position_to_angle = 0 ##  needs to be set by implementer
        self.Present_Position_range = -1  ##  needs to be set by implementer

        self.CW_Angle_Limit = 'CW_Angle_Limit'
        self.CCW_Angle_Limit = 'CCW_Angle_Limit'
        self.CW_Angle_Limit_Value = 0
        self.CCW_Angle_Limit_value = 0

        self.service_caller = service_caller
        self.config = config_class()
        self.motor_number = self.config.motor_id


    def call_service(self, address, value):
        """ call function to communicate with dynamixel"""
        return self.service_caller(self.motor_number, address, value)

    def set_wheel_mode(self):
        """ set the motor to wheel mode"""
        self.CW_Angle_Limit_Value = 0
        self.CCW_Angle_Limit_value = 0

        self.call_service(self.CW_Angle_Limit, self.CW_Angle_Limit_Value)
        self.call_service(self.CCW_Angle_Limit, self.CCW_Angle_Limit_Value)

        return True

    def set_joint_mode(self, CW_Angle, CWW_Angle):
        """ set the motor to joint mode

        Args:
            CW_Angle: must not be 0, is the CW angle limit
            CWW_Angle: must not be 0, is the CWW angle limit

        """

        if CW_Angle == 0 and CWW_Angle == 0:
            print('Invalid CW_Angle:', CW_Angle, 'CWW_Angle:', CWW_Angle)
            return False

        self.CW_Angle_Limit_Value = CW_Angle
        self.CCW_Angle_Limit_value = CWW_Angle

        self.call_service(self.CW_Angle_Limit, self.CW_Angle_Limit_Value)
        self.call_service(self.CCW_Angle_Limit, self.CCW_Angle_Limit_Value)

        return True

    def get_present_position(self):
        """ Get the present position of the Dynamixel motor"""

        ## not sure if return actually works
        #TODO
        motor_position_raw = self.call_service(self.Present_Position,0)
        return motor_position_raw*self.Present_Position_to_angle

class MX_64(MainMotor):
    def __init__(self, service_caller, config_class):
        super().__init__(service_caller, config_class)

        self.Present_Position_to_angle = 4095
        self.Present_Position_range = .088*3.14/180 # rad

    #EEPROM control table: CW_Angle_Limit, CCW_Angle_Limit
    # set both to 0 for wheel mode,
    # neither are 0 for joint mode
    # both are 4095 for multi-turn-mode


    def set_joint_mode(self, CW_Angle, CWW_Angle):
        """ set the motor to joint mode

        Args:
            CW_Angle: must not be 0 or 4095, is the CW angle limit
            CWW_Angle: must not be 0 or 4095, is the CWW angle limit

        """
        if CW_Angle == 4095 and CWW_Angle == 4095:
            print('Invalid CW_Angle:', CW_Angle, 'CWW_Angle:', CWW_Angle)
            return False

        super(MX_64, self.set_joint_mode(CW_Angle, CWW_Angle))


class MX_28(MainMotor):
    def __init__(self, service_caller, config_class):
        super().__init__(service_caller, config_class)
        self.Present_Position_to_angle = 4095
        self.Present_Position_range = .088*3.14/180 # rad

    #EEPROM control table: CW_Angle_Limit, CCW_Angle_Limit
    # set both to 0 for wheel mode,
    # neither are 0 for joint mode
    # both are 4095 for multi-turn-mode

    def set_joint_mode(self, CW_Angle, CWW_Angle):
        """ set the motor to joint mode

        Args:
            CW_Angle: must not be 0 or 4095, is the CW angle limit
            CWW_Angle: must not be 0 or 4095, is the CWW angle limit

        """
        if CW_Angle == 4095 and CWW_Angle == 4095:
            print('Invalid CW_Angle:', CW_Angle, 'CWW_Angle:', CWW_Angle)
            return False

        super(MX_28, self.set_joint_mode(CW_Angle, CWW_Angle))

class AX_12A(MainMotor):
    def __init__(self, service_caller, config_class):
        super().__init__(service_caller, config_class)

        self.Present_Position_to_angle = 1023
        self.Present_Position_range = .29*3.14/180 # rad

    #EEPROM control table: CW_Angle_Limit, CCW_Angle_Limit
    # set both to 0 for wheel mode, otherwise, is joint mode

