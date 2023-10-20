package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.constrainDegrees;
import static frc.robot.subsystems.wrist.WristConstants.degreesToRotations;
import static frc.robot.subsystems.wrist.WristConstants.kEncoderHomePosition;
import static frc.robot.subsystems.wrist.WristConstants.kMasterMotorConfiguration;
import static frc.robot.subsystems.wrist.WristConstants.kWristkA;
import static frc.robot.subsystems.wrist.WristConstants.kWristkD;
import static frc.robot.subsystems.wrist.WristConstants.kWristkG;
import static frc.robot.subsystems.wrist.WristConstants.kWristkI;
import static frc.robot.subsystems.wrist.WristConstants.kWristkP;
import static frc.robot.subsystems.wrist.WristConstants.kWristkS;
import static frc.robot.subsystems.wrist.WristConstants.kWristkV;
import static frc.robot.subsystems.wrist.WristConstants.rotationsToDegrees;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.lib.team6328.TunableNumber;

public class WristIOSparkMax implements WristIO {

    private final CANSparkMax m_master;

    private final RelativeEncoder m_encoder;
    private final SparkMaxPIDController m_pid;
    private final ArmFeedforward m_feedforward;

    private final TunableNumber wristkP = new TunableNumber("Wrist/WristkP", kWristkP);
    private final TunableNumber wristkI = new TunableNumber("Wrist/WristkI", kWristkI);
    private final TunableNumber wristkD = new TunableNumber("Wrist/WristkD", kWristkD);

    public WristIOSparkMax() {
        System.out.println("[Init] Creating WristIOSparkMax");
        m_master = SparkMaxFactory.createNEO(kMasterMotorConfiguration);
        m_encoder = m_master.getEncoder();
        m_pid = m_master.getPIDController();
        m_pid.setFeedbackDevice(m_encoder);
        m_encoder.setPosition(degreesToRotations(kEncoderHomePosition));
        BurnManager.burnFlash(m_master);

        SparkMaxFactory.configFramesLeaderOrFollower(m_master);
        SparkMaxFactory.configFramesPositionBoost(m_master);
        SparkMaxFactory.configFramesAbsoluteEncoderBoost(m_master);

        m_feedforward = new ArmFeedforward(kWristkS, kWristkG, kWristkV, kWristkA);
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(WristIOInputs inputs) {
        inputs.WristInternalPositionDeg = rotationsToDegrees(m_encoder.getPosition());
        inputs.WristInternalVelocityDegPerSec = rotationsToDegrees(m_encoder.getVelocity()) / 60.0;
        inputs.WristAppliedVolts = m_master.getAppliedOutput() * m_master.getBusVoltage();
        inputs.WristCurrentAmps = new double[] { m_master.getOutputCurrent() };
        inputs.WristTempCelsius = new double[] { m_master.getMotorTemperature() };

        // update tunables
        if (wristkP.hasChanged(wristkP.hashCode()) || wristkI.hasChanged(wristkI.hashCode())
                || wristkD.hasChanged(wristkD.hashCode())) {
            m_pid.setP(wristkP.get());
            m_pid.setI(wristkI.get());
            m_pid.setD(wristkD.get());
        }
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
        targetAngleDegrees = constrainDegrees(targetAngleDegrees);
        double targetRotation = degreesToRotations(targetAngleDegrees);
        //m_pid.setReference(targetRotation, ControlType.kPosition, 0, ff);
        m_pid.setReference(targetRotation, ControlType.kPosition, 0, ff);
    }

    public void resetSensorPosition(double angleDegrees) {
        m_encoder.setPosition(degreesToRotations(angleDegrees));
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
