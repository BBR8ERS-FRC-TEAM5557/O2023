package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.lib.team6328.TunableNumber;

import static frc.robot.subsystems.swerve.SwerveConstants.*;


public class ModuleIOSparkMax implements ModuleIO {

    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;

    private final SparkMaxPIDController m_angleMotorPID;
    private final SparkMaxPIDController m_driveMotorPID;

    private final RelativeEncoder m_angleMotorEncoder;
    private final RelativeEncoder m_driveMotorEncoder;

    private CANCoder m_absoluteEncoder;

    private SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

    private final TunableNumber driveKp = new TunableNumber("Swerve/DriveKp", kDrivekP);
    private final TunableNumber driveKi = new TunableNumber("Swerve/DriveKi", kDrivekI);
    private final TunableNumber driveKd = new TunableNumber("Swerve/DriveKd", kDrivekD);
    private final TunableNumber angleKp = new TunableNumber("Swerve/AngleKp", kAnglekP);
    private final TunableNumber angleKi = new TunableNumber("Swerve/AngleKi", kAnglekI);
    private final TunableNumber angleKd = new TunableNumber("Swerve/AngleKd", kAnglekD);

    private final double m_angleOffsetDeg;
    private double resetIteration;

    public ModuleIOSparkMax(int moduleNumber, int driveMotorID, int angleMotorID, int canCoderID,
            double angleOffsetDeg) {
        System.out.println("[Init] Creating ModuleIOSparkMax");
        this.m_angleOffsetDeg = angleOffsetDeg;

        configAngleEncoder(canCoderID);
        configAngleMotor(angleMotorID);
        configDriveMotor(driveMotorID);

        m_angleMotorPID = m_angleMotor.getPIDController();
        m_driveMotorPID = m_driveMotor.getPIDController();

        m_angleMotorEncoder = m_angleMotor.getEncoder();
        m_driveMotorEncoder = m_driveMotor.getEncoder();
    }

    private void configAngleEncoder(int id) {
        m_absoluteEncoder = new CANCoder(id);

        m_absoluteEncoder.configFactoryDefault();

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.sensorDirection = kCanCoderInverted;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = this.m_angleOffsetDeg;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        m_absoluteEncoder.configAllSettings(config);
    }

    private void configDriveMotor(int id) {
        m_driveMotor = SparkMaxFactory.createNEO(id, kDriveMotorConfiguration);
    }

    private void configAngleMotor(int id) {
        m_angleMotor = SparkMaxFactory.createNEO(id, kAngleMotorConfiguration);

        m_angleMotorPID.setPositionPIDWrappingMinInput(0.0);
        m_angleMotorPID.setPositionPIDWrappingMaxInput(kAngleGearReduction);
        m_angleMotorPID.setPositionPIDWrappingEnabled(true);

        var resetSuccesful = false;
        int resetIteration = 0;
        while(!resetSuccesful && resetIteration < 5)
            resetSuccesful = resetToAbsolute();
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveDistanceMeters = rotationsToMeters(
                m_driveMotorEncoder.getPosition(),
                kWheelCircumference,
                kDriveGearReduction);
        inputs.driveVelocityMetersPerSec = rpmToMPS(
                m_driveMotorEncoder.getVelocity(),
                kWheelCircumference,
                kDriveGearReduction);
        inputs.driveAppliedVolts = m_driveMotor.getAppliedOutput() * m_driveMotor.getBusVoltage();
        inputs.driveCurrentAmps = new double[] { m_driveMotor.getOutputCurrent() };
        inputs.driveTempCelsius = new double[] { m_driveMotor.getMotorTemperature() };

        inputs.angleAbsolutePositionRad = Units.degreesToRadians(m_absoluteEncoder.getAbsolutePosition());
        inputs.angleInternalPositionRad = rotationsToRadians(m_angleMotorEncoder.getPosition(), kAngleGearReduction);
        inputs.angleInternalVelocityRadPerSec = m_angleMotorEncoder.getVelocity() * (Math.PI * 2.0) / (60.0 * kAngleGearReduction); 
        inputs.angleAppliedVolts = m_angleMotor.getAppliedOutput() * m_angleMotor.getBusVoltage();
        inputs.angleCurrentAmps = new double[] { m_angleMotor.getOutputCurrent() };
        inputs.angleTempCelsius = new double[] { m_angleMotor.getMotorTemperature() };

        // update tunables
        if (driveKp.hasChanged(driveKp.hashCode()) || driveKi.hasChanged(driveKi.hashCode()) || driveKd.hasChanged(driveKd.hashCode())
            || angleKp.hasChanged(angleKp.hashCode()) || angleKi.hasChanged(angleKi.hashCode())|| angleKd.hasChanged(angleKd.hashCode())) {
            m_driveMotorPID.setP(driveKp.get());
            m_driveMotorPID.setI(driveKi.get());
            m_driveMotorPID.setD(driveKd.get());
            m_angleMotorPID.setP(angleKp.get());
            m_angleMotorPID.setI(angleKi.get());
            m_angleMotorPID.setD(angleKd.get());
        }

    }

    /** Run the drive motor at the specified percentage of full power. */
    public void setDriveMotorPercentage(double percentage) {
        m_driveMotorPID.setReference(percentage, ControlType.kDutyCycle);
    }

    public void setDriveVoltage(double voltage) {
        m_driveMotorPID.setReference(voltage, ControlType.kVoltage);
    }

    /** Run the drive motor at the specified velocity. */
    public void setDriveVelocity(double velocity) {
        double rotationsPerMinute = mpsToRPM(
            velocity,
            kWheelCircumference,
            kDriveGearReduction);

        double feedforward = m_driveFeedforward.calculate(velocity);

        m_driveMotorPID.setReference(rotationsPerMinute, ControlType.kVelocity, 0, feedforward);
    }

    public void setAngleVoltage(double voltage) {
        m_angleMotorPID.setReference(voltage, ControlType.kVoltage);
    }

    /** Run the turn motor to the specified angle. */
    public void setAnglePosition(double radians) {
        double desiredAngleRotations = radiansToRotations(radians, kAngleGearReduction);

        if (m_angleMotorEncoder.getVelocity() < kAbsoluteResetMaxOmega) {
            if (++resetIteration >= kAbsoluteResetIterations) {
                resetIteration = 0;
                resetToAbsolute();
            }
        } else {
            resetIteration = 0;
        }

        m_angleMotorPID.setReference(desiredAngleRotations, ControlType.kPosition);
    }

    /** Enable or disable brake mode on the drive motor. */
    public void setDriveBrakeMode(boolean enable) {
        if(enable) {
            m_driveMotor.setIdleMode(IdleMode.kBrake);
        } else {
            m_driveMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    /** Enable or disable brake mode on the turn motor. */
    public void setAngleBrakeMode(boolean enable) {
        if(enable) {
            m_angleMotor.setIdleMode(IdleMode.kBrake);
        } else {
            m_angleMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public boolean resetToAbsolute() {
        if(m_absoluteEncoder.getLastError() == ErrorCode.OK) {
            double absoluteAngle = getCanCoderRotation().getRadians();
            m_angleMotorEncoder.setPosition(radiansToRotations(absoluteAngle, kAngleGearReduction)); //fix units
            return true;
        }
        return false;
    }

    private Rotation2d getCanCoderRotation() {
        return Rotation2d.fromDegrees(m_absoluteEncoder.getAbsolutePosition());
    }

}
