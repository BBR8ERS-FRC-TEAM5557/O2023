package frc.robot.subsystems.swerve;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.team5557.factory.SparkMaxFactory.PIDConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.SparkMaxConfiguration;
import frc.robot.util.SwerveSetpointGenerator.KinematicLimits;

public class SwerveConstants {

    public static final String kSubsystemName = "Swerve";

    public static final double kTrackWidth = Units.inchesToMeters(26.0);
    public static final double kWheelBase = Units.inchesToMeters(26.0);

    /* Swerve Profiling Values */
    public static final double kMaxSpeed = Units.feetToMeters(14.5); // meters per second
    public static final double kMaxOmega = 11.5; //radians per second
    public static final double kMaxAttainableSpeed = kMaxSpeed * 0.85; // Max out at 85% to make sure speeds are attainable (4.6 mps)
    public static final double kMaxAcceleration = 3.0; // m/s^2
    public static final double kMaxAttainableAcceleration = kMaxAcceleration * 0.85;


    public static final boolean kCanCoderInverted = false;
    public static final int kAbsoluteResetIterations = 100;
    public static final double kAbsoluteResetMaxOmega = 4.0; //must rotate at less than a degree per second

    public static final double kAngleGearReduction = 150.0 / 7.0;
    public static final double kDriveGearReduction = (50.0 / 14.0) * (17.0 / 27.0) * ( 45.0 / 15.0);
    public static final double kWheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

    public static final double kAnglekP = 0.3; // FIXME: error(rotations) * kP = volts -> kp = 0.6 = 12.0 volts / 20 rotations
    public static final double kAnglekI = 0.0;
    public static final double kAnglekD = 0.0;

    public static final double kDrivekP = 0.000; // FIXME: error(rpm) * kP = volts -> kp = 0.6 = 12.0 volts / 5000 rpm
    public static final double kDrivekI = 0.0;
    public static final double kDrivekD = 0.0;

    public static final double kDrivekS = 0.0;
    public static final double kDrivekV = 0.0;
    public static final double kDrivekA = 0.0;

    public static final double kTranslationkP = 5.0; // error(meters) * kP = velocity m/s  -> kp = velocity / error --> 4.5 / 1.0
    public static final double kTranslationkI = 0.0;
    public static final double kTranslationkD = 0.0;

    public static final double kRotationkP = 4.0;
    public static final double kRotationkI = 0.0;
    public static final double kRotationkD = 0.0;

    public static final double kSnapMaxOmega = kMaxOmega * 0.85;
    public static final double kSnapMaxAlpha = kSnapMaxOmega / 0.75;


    public static final KinematicLimits kUncappedLimits = new KinematicLimits();
    static {
            kUncappedLimits.kMaxDriveVelocity = kMaxSpeed;
            kUncappedLimits.kMaxDriveAcceleration = Double.MAX_VALUE;
            kUncappedLimits.kMaxAzimuthVelocity = 20.0; //rad/s
            kUncappedLimits.kMaxAngularVelocity = kMaxOmega * 0.75;
    }

    public static final KinematicLimits kScoringLimits = new KinematicLimits();
    static {
            kScoringLimits.kMaxDriveVelocity = 2.0;
            kScoringLimits.kMaxDriveAcceleration = 20.0;
            kScoringLimits.kMaxAzimuthVelocity = 20.0;
            kScoringLimits.kMaxAngularVelocity = Math.PI;  // Rad/Sec
    }

    public static final KinematicLimits kPathFollowingLimits = new KinematicLimits();
    static {
            kPathFollowingLimits.kMaxDriveVelocity = kMaxAttainableSpeed;
            kPathFollowingLimits.kMaxDriveAcceleration = kMaxAttainableAcceleration;
            kPathFollowingLimits.kMaxAzimuthVelocity = 20.0;
            kPathFollowingLimits.kMaxAngularVelocity = 2 * Math.PI;
    }

    public static PIDConfiguration kDrivePIDConfiguration = new PIDConfiguration();
    static {
        kDrivePIDConfiguration.kP = kDrivekP;
        kDrivePIDConfiguration.kI = kDrivekI;
        kDrivePIDConfiguration.kD = kDrivekD;
    }

    public static SparkMaxConfiguration kDriveMotorConfiguration = new SparkMaxConfiguration();
    static {
        kDriveMotorConfiguration.label = "Swerve Drive Motor";

        kDriveMotorConfiguration.pid = kDrivePIDConfiguration;

        kDriveMotorConfiguration.kShouldInvert = false;
        kDriveMotorConfiguration.kVoltageCompensation = 12.0;
        kDriveMotorConfiguration.kSmartCurrentLimit = 50.0;
        kDriveMotorConfiguration.kIdleMode = IdleMode.kBrake;
    }

    public static PIDConfiguration kAnglePIDConfiguration = new PIDConfiguration();
    static {
        kAnglePIDConfiguration.kP = kAnglekP;
        kAnglePIDConfiguration.kI = kAnglekI;
        kAnglePIDConfiguration.kD = kAnglekD;
    }

    public static SparkMaxConfiguration kAngleMotorConfiguration = new SparkMaxConfiguration();
    static {
        kAngleMotorConfiguration.pid = kAnglePIDConfiguration;

        kAngleMotorConfiguration.kShouldInvert = true;
        kAngleMotorConfiguration.kVoltageCompensation = 12.0;
        kAngleMotorConfiguration.kSmartCurrentLimit = 20.0;
        kAngleMotorConfiguration.kIdleMode = IdleMode.kBrake;
    }


    public static final Translation2d[] kSwerveModuleLocations = {
        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
    };



    /**
     * @param counts    NEO Rotations
     * @param gearRatio Gear Ratio between NEO and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double rotationsToDegrees(double rotations, double gearRatio) {
        return rotations * (360.0 / gearRatio);
    }

    /**
     * This is a commonly used function when setting the range of a radian measurement from [-pi, pi)
     * to [0,2pi]. This is requred a lot when ensuring that the number being sent in to the NEO's onboard
     * angle PID Controller is within the range necessary. Note that this also performs modular dision first
     * on the angle which allows for an angle like -3pi be sent in, and return 1pi.
     * @param radians - the 
     * @return the same angle in the range [0,2pi)
     */
    public static double convertPiPositive(double radians) {
        double angle = radians;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    /**
    * @param velocity Velocity MPS
    * @param circumference Circumference of Wheel
    * @param gearRatio Gear Ratio between Motor and Mechanism (set to 1 for Falcon RPM)
    * @return Falcon Velocity Counts
    */
    public static double mpsToRPM(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60.0) / circumference);
        return wheelRPM * gearRatio;
    }

    /**
     * @param velocityrotations Motor Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between motor and mechanism
     * @return Wheel Meters per second
     */
    public static double rpmToMPS(double velocityrotations, double circumference, double gearRatio) {
        return rotationsToMeters(velocityrotations, circumference, gearRatio) / 60.0;
    }

    /**
     * @param counts NEO rotations
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Motor and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
        double wheelRotations = rotations / gearRatio;
        return (wheelRotations * circumference);
    }

    public static double radiansToRotations(double radians, double gearRatio) {
        return (radians / (2 * Math.PI)) * gearRatio;
    }

    public static double rotationsToRadians(double rotations, double gearRatio) {
        return (rotations * (2 * Math.PI)) / gearRatio;
    }

}
