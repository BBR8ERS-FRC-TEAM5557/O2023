package frc.robot.subsystems.wrist;

import static frc.robot.subsystems.wrist.WristConstants.constrainDegrees;
import static frc.robot.subsystems.wrist.WristConstants.kCruiseVelocity;
import static frc.robot.subsystems.wrist.WristConstants.kEncoderHomePosition;
import static frc.robot.subsystems.wrist.WristConstants.kHomeAmpsThreshold;
import static frc.robot.subsystems.wrist.WristConstants.kHomeVoltage;
import static frc.robot.subsystems.wrist.WristConstants.kPadding;
import static frc.robot.subsystems.wrist.WristConstants.kTimeToCruise;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.TunableNumber;
import frc.robot.RobotContainer;
import frc.robot.subsystems.wrist.WristIO.WristIOInputs;
import frc.robot.util.Util;

public class Wrist extends SubsystemBase {
    private final WristIO m_io;
    private final WristIOInputs m_inputs = new WristIOInputs();

    private ControlMode m_mode = ControlMode.OPEN_LOOP;

    private TrapezoidProfile.Constraints m_constraints =
            new TrapezoidProfile.Constraints(kCruiseVelocity, (kCruiseVelocity / kTimeToCruise));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints, m_goal);
    private double m_profileTimestamp = 0.0;
    private double m_demand = 0.0;

    public final TunableNumber cruiseVelocity =
            new TunableNumber("Wrist/cruiseVelocity", kCruiseVelocity);
    public final TunableNumber desiredTimeToSpeed =
            new TunableNumber("Wrist/desiredTimeToSpeed", kTimeToCruise);

    public enum ControlMode {
        OPEN_LOOP, VOLTAGE, POSITION, MOTION_PROFILE
    }

    public Wrist(WristIO io) {
        System.out.println("[Init] Creating Wrist");
        this.m_io = io;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Wrist");
        shuffleboardTab.addNumber("Position Internal", () -> Util.truncate(getState().position, 2))
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab
                .addNumber("Position Absolute",
                        () -> Util.truncate(m_inputs.WristAbsolutePositionDeg, 2))
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Velocity", () -> Util.truncate(getState().velocity, 2))
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Demand", () -> Util.truncate(m_demand, 2))
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Output", () -> Util.truncate(m_inputs.WristAppliedVolts, 2))
                .withWidget(BuiltInWidgets.kGraph);

        shuffleboardTab.addString("Control Mode", () -> getControlMode().name());
        shuffleboardTab.addString("Command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "NONE");
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.getInstance().processInputs("Wrist", m_inputs);
        Logger.getInstance().recordOutput("Wrist/Demand", m_demand);

        if (m_mode == ControlMode.OPEN_LOOP) {
            m_io.setPercent(m_demand);
        } else if (m_mode == ControlMode.VOLTAGE) {
            m_io.setVoltage(m_demand);
        } else if (m_mode == ControlMode.POSITION) {
            m_io.setAngleDegrees(m_demand, 0.0);
        } else {
            m_setpoint = m_profile.calculate(Timer.getFPGATimestamp() - m_profileTimestamp);
            m_io.setAngleDegrees(m_setpoint.position, m_setpoint.velocity);

            Logger.getInstance().recordOutput("Wrist/Setpoint", m_setpoint.position);
        }

        if (cruiseVelocity.hasChanged(cruiseVelocity.hashCode())
                || desiredTimeToSpeed.hasChanged(desiredTimeToSpeed.hashCode())) {
            m_constraints = new TrapezoidProfile.Constraints(cruiseVelocity.get(),
                    (cruiseVelocity.get() / desiredTimeToSpeed.get()));
        }
    }

    private synchronized void runOpenLoop(double percent) {
        if (m_mode != ControlMode.OPEN_LOOP) {
            m_mode = ControlMode.OPEN_LOOP;
        }
        m_demand = percent;
    }

    private synchronized void runVoltage(double voltage) {
        if (m_mode != ControlMode.VOLTAGE) {
            m_mode = ControlMode.VOLTAGE;
        }
        m_demand = voltage;
    }

    private synchronized void runPosition(double degrees) {
        if (m_mode != ControlMode.POSITION) {
            m_mode = ControlMode.POSITION;
        }
        m_demand = constrainDegrees(degrees);
    }

    private synchronized void runMotionProfile(double degrees) {
        if (m_mode != ControlMode.MOTION_PROFILE || m_goal.position != degrees) {
            m_mode = ControlMode.MOTION_PROFILE;
            m_goal = new TrapezoidProfile.State(constrainDegrees(degrees), 0.0);
            m_profile = new TrapezoidProfile(m_constraints, m_goal, getState());
            m_profileTimestamp = Timer.getFPGATimestamp();
            m_demand = constrainDegrees(degrees);
        }
    }

    public synchronized TrapezoidProfile.State getState() {
        //return new TrapezoidProfile.State(m_inputs.WristAbsolutePositionDeg, m_inputs.WristAbsoluteVelocityDegPerSec);
        return new TrapezoidProfile.State(m_inputs.WristInternalPositionDeg, m_inputs.WristInternalVelocityDegPerSec);
    }


    // Command Building Bloacks
    public CommandBase runWristOpenLoop(DoubleSupplier percent) {
        return new RunCommand(() -> runOpenLoop(percent.getAsDouble()), this);
    }

    public Command homeWrist() {
        return Commands
                .sequence(new InstantCommand(() -> m_io.shouldEnableUpperLimit(false)),
                        new RunCommand(() -> runVoltage(kHomeVoltage), this)
                                .until(() -> m_inputs.WristCurrentAmps[0] > kHomeAmpsThreshold),
                        new InstantCommand(() -> m_io.resetSensorPosition(kEncoderHomePosition)),
                        setWristAngleProfiled(kEncoderHomePosition))
                .finallyDo(
                        interupted -> new InstantCommand(() -> m_io.shouldEnableUpperLimit(true)));
    }

    public Command setWristAngle(double targetAngle) {
        return new InstantCommand(() -> runPosition(targetAngle), this);
    }

    public Command setWristAngleProfiled(double targetAngle) {
        return new InstantCommand(() -> runMotionProfile(targetAngle), this);
    }

    // Wait decorator commands
    public Command tuckWaitCommand(double angle) {
        return new WaitUntilCommand(() -> getState().position > angle);
    }

    public Command extendWaitCommand(double angle) {
        return new WaitUntilCommand(() -> getState().position < angle);
    }

    public Command epsilonWaitCommand(double angle) {
        return new WaitUntilCommand(() -> Math.abs(getState().position - angle) < kPadding);
    }

    public Command epsilonWaitCommand() {
        return new WaitUntilCommand(
                () -> Math.abs(getState().position - m_goal.position) < kPadding);
    }

    public ControlMode getControlMode() {
        return m_mode;
    }
}
