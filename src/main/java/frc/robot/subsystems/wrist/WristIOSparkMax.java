package frc.robot.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.team5557.factory.SparkMaxFactory;
import static frc.robot.subsystems.wrist.WristConstants.*;

public class WristIOSparkMax implements WristIO {

    private final CANSparkMax m_master;

    private final RelativeEncoder m_encoder;
    private final AbsoluteEncoder m_absoluteEncoder;
    private final SparkMaxPIDController m_pid;

    private final ArmFeedforward m_feedforward;

    public WristIOSparkMax() {
        System.out.println("[Init] Creating WristIOSparkMax");
        m_master = SparkMaxFactory.createNEO(12, kMasterMotorConfiguration);
        m_encoder = m_master.getEncoder();
        m_pid = m_master.getPIDController();

        m_absoluteEncoder = m_master.getAbsoluteEncoder(Type.kDutyCycle);
        m_absoluteEncoder.setInverted(false);
        m_absoluteEncoder.setPositionConversionFactor(360.0);
        m_absoluteEncoder.setVelocityConversionFactor(360.0);
        m_absoluteEncoder.setZeroOffset(0.0);

        m_encoder.setPosition(degreesToRotations(m_absoluteEncoder.getPosition() - 180.0));

        SparkMaxFactory.configFramesLeaderOrFollower(m_master);
        SparkMaxFactory.configFramesPositionBoost(m_master);

        m_feedforward = new ArmFeedforward(kWristkS, kWristkG, kWristkV, kWristkA);
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(WristIOInputs inputs) {
        inputs.WristInternalPositionDeg = rotationsToDegrees(m_encoder.getPosition());
        inputs.WristInternalVelocityDegPerSec = rotationsToDegrees(m_encoder.getVelocity()) / 60.0;
        inputs.WristAbsolutePositionDeg = m_absoluteEncoder.getPosition() - 180.0;
        inputs.WristAbsoluteVelocityDegPerSec = m_absoluteEncoder.getVelocity();
        inputs.WristAppliedVolts = m_master.getAppliedOutput() * m_master.getBusVoltage();
        inputs.WristCurrentAmps = new double[] {m_master.getOutputCurrent()};
        inputs.WristTempCelsius = new double[] {m_master.getMotorTemperature()};
    }

    /** Run the Wrist open loop at the specified voltage. */
    public void setVoltage(double volts) {
        m_pid.setReference(volts, ControlType.kVoltage);
    }

    public void setPercent(double percent) {
        m_pid.setReference(percent, ControlType.kDutyCycle);
    }

    public void setAngleDegrees(double targetAngleDegrees, double targetVelocityDegreesPerSec) {
        double ff = m_feedforward.calculate(targetAngleDegrees, targetVelocityDegreesPerSec);
        double targetAngleRotations = degreesToRotations(targetAngleDegrees);
        m_pid.setReference(targetAngleRotations, ControlType.kPosition, 0, ff);
    }

    public void resetSensorPosition(double angleDegrees) {
        m_encoder.setPosition(angleDegrees);
    }

    public void brakeOff() {
        m_master.setIdleMode(IdleMode.kCoast);
    }

    public void brakeOn() {
        m_master.setIdleMode(IdleMode.kBrake);
    }

    public void shouldEnableUpperLimit(boolean value) {
        m_master.enableSoftLimit(SoftLimitDirection.kForward, value);
    }

    public void shouldEnableLowerLimit(boolean value) {
        m_master.enableSoftLimit(SoftLimitDirection.kReverse, value);
    }
}
