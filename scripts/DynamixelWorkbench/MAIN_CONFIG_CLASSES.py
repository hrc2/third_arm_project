

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

        self.CW_Angle_Limit = 'CW_Angle_Limit'
        self.CCW_Angle_Limit = 'CCW_Angle_Limit'
        self.CW_Angle_Limit_Value = 0
        self.CCW_Angle_Limit_value = 0

        self.service_caller = service_caller
        self.config = config_class()
        self.motor_number = self.config.motor_id


    def call_service(self, address, value):
        """ call function to communicate with dynamixel"""
        self.service_caller(self.motor_number, address, value)

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

        self.scall_service(self.CW_Angle_Limit, self.CW_Angle_Limit_Value)
        self.call_service(self.CCW_Angle_Limit, self.CCW_Angle_Limit_Value)

        return True


class MX_64(MainMotor):
    def __init__(self, service_caller, motor_number, config_class):
        super(MX_64, self.__init__(service_caller, motor_number, config_class))

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
    def __init__(self, service_caller, motor_number, config_class):
        super(MX_28, self.__init__(service_caller, motor_number, config_class))

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
    def __init__(self, service_caller, motor_number, config_class):
        super(AX_12A, self.__init__(service_caller, motor_number, config_class))

    #EEPROM control table: CW_Angle_Limit, CCW_Angle_Limit
    # set both to 0 for wheel mode, otherwise, is joint mode

