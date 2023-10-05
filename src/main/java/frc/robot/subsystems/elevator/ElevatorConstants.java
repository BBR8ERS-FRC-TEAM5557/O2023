package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import frc.lib.team5557.factory.SparkMaxFactory.PIDConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.SoftLimitsConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.SparkMaxConfiguration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;

public class ElevatorConstants {
    public static final double kGearReduction = 15.0;
    public static final double kSprocketPitchDiameter = 1.75; //inches
    public static final double kRotationsPerInch = kGearReduction / (kSprocketPitchDiameter * Math.PI);

    public static final double kPadding = 0.5; // inches
    public static final double kCruiseVelocity = 50.0; // inches/sec
    public static final double kTimeToCruise = 0.25; // sec

    public static final double kEncoderHomePosition = 0.0; //inches
    public static final double kMinHeight = 0.0; //inches
    public static final double kMaxHeight = 30.0; //inches

    public static final double kHomeVoltage = 4.0;
    public static final double kHomeAmpsThreshold = 20.0;

    public static final double kElevatorkP = 0.05;
    public static final double kElevatorkI = 0.0;
    public static final double kElevatorkD = 0.0;

    public static final double kElevatorkS = 0.0;
    public static final double kElevatorkG = 0.0;
    public static final double kElevatorkV = 0.0;
    public static final double kElevatorkA = 0.0;
    
    public static final SoftLimitsConfiguration kLimitConfiguration = new SoftLimitsConfiguration();
    static {
        kLimitConfiguration.kUpperLimit = Double.NaN;
        kLimitConfiguration.kLowerLimit = inchesToRotations(kMinHeight);
    }

    public static final PIDConfiguration kPIDConfiguration = new PIDConfiguration();
    static {
        kPIDConfiguration.kP = kElevatorkP;
        kPIDConfiguration.kI = kElevatorkI;
        kPIDConfiguration.kD = kElevatorkD;
        kPIDConfiguration.kF = 0.0;
        kPIDConfiguration.kTolerance = inchesToRotations(kPadding);
    }

    public static final SparkMaxConfiguration kMasterMotorConfiguration = new SparkMaxConfiguration();
    static {
        kMasterMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, 31);
        kMasterMotorConfiguration.label = "Elevator Master";

        kMasterMotorConfiguration.pid = kPIDConfiguration;
        kMasterMotorConfiguration.limits = kLimitConfiguration;

        kMasterMotorConfiguration.kVoltageCompensation = 12.0;
        kMasterMotorConfiguration.kSmartCurrentLimit = 40.0;
        kMasterMotorConfiguration.kOpenLoopRampRate = 1.0;
        kMasterMotorConfiguration.kClosedLoopRampRate = 0.25;
        kMasterMotorConfiguration.kShouldInvert = false;
        kMasterMotorConfiguration.kIdleMode = IdleMode.kBrake;

    }

    public static double rotationsToInches(double rotations) {
        return rotations / kRotationsPerInch;
    }

    public static double inchesToRotations(double inches) {
        return inches * kRotationsPerInch;
    }

    public static double constrainInches(double inches) {
        return MathUtil.clamp(inches, kMinHeight, kMaxHeight);
    }
}
