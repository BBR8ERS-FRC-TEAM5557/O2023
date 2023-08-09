package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.TunableNumber;
import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import frc.robot.util.Util;

public class Elevator extends SubsystemBase {
    private final ElevatorIO m_io;
    private final ElevatorIOInputs m_inputs = new ElevatorIOInputs();

    private ControlMode m_mode = ControlMode.OPEN_LOOP;
    private TrapezoidProfile.Constraints m_constraints =
            new TrapezoidProfile.Constraints(kCruiseVelocity, (kCruiseVelocity / kTimeToCruise));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    private TrapezoidProfile m_profile = new TrapezoidProfile(m_constraints, m_goal);
    private double m_profileTimestamp = 0.0;
    private double m_demand = 0.0;

    public final TunableNumber cruiseVelocity =
            new TunableNumber("Elevator/cruiseVelocity", kCruiseVelocity);
    public final TunableNumber desiredTimeToSpeed =
            new TunableNumber("Elevator/desiredTimeToSpeed", kTimeToCruise);

    public enum ControlMode {
        OPEN_LOOP, VOLTAGE, POSITION, MOTION_PROFILE
    }

    public Elevator(ElevatorIO io) {
        System.out.println("[Init] Creating Elevator");
        this.m_io = io;

        // Automatic Home Trigger
        new Trigger(() -> (m_mode == ControlMode.POSITION || m_mode == ControlMode.MOTION_PROFILE)
                && (Util.epsilonEquals(m_demand, kEncoderHomePosition, 1.0))
                && (Util.epsilonEquals(m_inputs.ElevatorHeightInches, kEncoderHomePosition, 1.0)))
                        .onTrue(homeElevator());
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.getInstance().processInputs("Elevator", m_inputs);
        Logger.getInstance().recordOutput("Elevator/Demand", m_demand);

        if (m_mode == ControlMode.OPEN_LOOP) {
            m_io.setPercent(m_demand);
        } else if (m_mode == ControlMode.VOLTAGE) {
            m_io.setVoltage(m_demand);
        } else if (m_mode == ControlMode.POSITION) {
            m_io.setHeightInches(m_demand, 0.0);
        } else {
            m_setpoint = m_profile.calculate(Timer.getFPGATimestamp() - m_profileTimestamp);
            m_io.setHeightInches(m_setpoint.position, m_setpoint.velocity);
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

    private synchronized void runPosition(double inches) {
        if (m_mode != ControlMode.POSITION) {
            m_mode = ControlMode.POSITION;
        }
        m_demand = constrainInches(inches);
    }

    private synchronized void runMotionProfile(double inches) {
        if (m_mode != ControlMode.MOTION_PROFILE || m_goal.position != inches) {
            m_mode = ControlMode.MOTION_PROFILE;
            m_goal = new TrapezoidProfile.State(constrainInches(inches), 0.0);
            m_profile = new TrapezoidProfile(m_constraints, m_goal, getState());
            m_profileTimestamp = Timer.getFPGATimestamp();
            m_demand = constrainInches(inches);
        }
    }

    public synchronized TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(m_inputs.ElevatorHeightInches,
                m_inputs.ElevatorVelocityInchesPerSecond);
    }

    // Elevator Control Building Blocks
    public Command runElevatorOpenLoop(DoubleSupplier percent) {
        return new RunCommand(() -> runOpenLoop(percent.getAsDouble()), this);
    }

    public Command homeElevator() {
        return Commands
                .sequence(new InstantCommand(() -> m_io.shouldEnableLowerLimit(false)),
                        new RunCommand(() -> runVoltage(-kHomeVoltage), this)
                                .until(() -> m_inputs.ElevatorCurrentAmps[0] > kHomeAmpsThreshold),
                        new InstantCommand(() -> m_io.resetSensorPosition(kEncoderHomePosition)),
                        setElevatorHeightProfiled(kEncoderHomePosition))
                .finallyDo(
                        interupted -> new InstantCommand(() -> m_io.shouldEnableLowerLimit(true)));
    }

    public Command setElevatorHeight(double targetHeight) {
        return new InstantCommand(() -> runPosition(targetHeight), this);
    }

    public Command setElevatorHeightProfiled(double targetHeight) {
        return new InstantCommand(() -> runMotionProfile(targetHeight), this);
    }

    // Wait Command Decorators
    public Command tuckWaitCommand(double length) {
        return new WaitUntilCommand(() -> getState().position < length);
    }

    public Command extendWaitCommand(double length) {
        return new WaitUntilCommand(() -> getState().position > length);
    }

    public Command epsilonWaitCommand(double length) {
        return new WaitUntilCommand(() -> Math.abs(getState().position - length) < kPadding);
    }

    public Command epsilonWaitCommand() {
        return new WaitUntilCommand(
                () -> Math.abs(getState().position - m_goal.position) < kPadding);
    }
}
