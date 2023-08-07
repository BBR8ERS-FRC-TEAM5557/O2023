package frc.robot.subsystems.roller;

import com.revrobotics.CANSparkMax.IdleMode;

import frc.lib.team5557.factory.SparkMaxFactory.SparkMaxConfiguration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;

public class RollerConstants {
    
    public static SparkMaxConfiguration kRollerMotorConfiguration = new SparkMaxConfiguration();
    static {
        kRollerMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, 0);
        kRollerMotorConfiguration.label = "Roller Motor";

        kRollerMotorConfiguration.kVoltageCompensation = 30.0;
        kRollerMotorConfiguration.kOpenLoopRampRate = 1.0;
        kRollerMotorConfiguration.kShouldInvert = false;
        kRollerMotorConfiguration.kVoltageCompensation = 10.0;
        kRollerMotorConfiguration.kIdleMode = IdleMode.kBrake;
    }
}
