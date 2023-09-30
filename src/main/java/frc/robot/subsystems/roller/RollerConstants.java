package frc.robot.subsystems.roller;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.lib.team5557.factory.SparkMaxFactory.SparkMaxConfiguration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;
import frc.robot.Constants.RobotMap;

public class RollerConstants {
    
    public static SparkMaxConfiguration kRollerMotorConfiguration = new SparkMaxConfiguration();
    static {
        kRollerMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, RobotMap.kRollerMotor);
        kRollerMotorConfiguration.label = "Roller Motor";

        kRollerMotorConfiguration.kSmartCurrentLimit = 30.0;
        kRollerMotorConfiguration.kOpenLoopRampRate = 0.25;
        kRollerMotorConfiguration.kShouldInvert = false;
        kRollerMotorConfiguration.kVoltageCompensation = 12.0;
        kRollerMotorConfiguration.kIdleMode = IdleMode.kBrake;
    }
}
