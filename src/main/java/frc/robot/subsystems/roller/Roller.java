package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.roller.RollerIO.RollerIOInputs;

public class Roller extends SubsystemBase {

    private final RollerIO m_io;
    private final RollerIOInputs m_inputs = new RollerIOInputs();

    private State currentState = State.DO_NOTHING;

    public Roller(RollerIO io) {
        System.out.println("[Init] Creating Roller");
        this.m_io = io;
    }

    @Override
    public void periodic() {
        m_io.setRollerVoltage(this.currentState.getMotorVoltage());

        Logger.getInstance().processInputs("Roller", m_inputs);

        Logger.getInstance().recordOutput("Current State", currentState.toString());
        // Logger.getInstance().recordOutput("isStalled", getIsStalled());
    }

    public void setRollerState(State desState) {
        this.currentState = desState;
    }


    /*
     * // this needs to be called repeatedly because it uses a filter. // I.E calling this once will
     * not return an accurate result // need to call this in an isFinished or a .until() public
     * boolean isStalled() { // var filteredCurrent =
     * StallDetectionFilter.calculate(Math.abs(inputs.intakeStatorCurrent)); // var velocity =
     * inputs.intakeVelocityRPM;
     * 
     * // double minStallCurrent = 1; // double maxStallVelocity = 50;
     * 
     * // return (filteredCurrent >= minStallCurrent) && (velocity <= maxStallVelocity);
     * 
     * // FIXME: need to check these numbers
     * 
     * return (filteredVelocity <= velocityThreshold.get() && (filteredStatorCurrent >=
     * currentThreshold.get() || filteredStatorCurrent <= -2)); }
     */

    public enum State {
        INTAKING_CUBE(10.0), EJECT_CUBE(-5.0), HOLD_CUBE(2.0),

        INTAKING_CONE(10.0), EJECT_CONE(-5.0), HOLD_CONE(2.0),

        DO_NOTHING(0.0), IDLE(1.0);

        private double motorVoltage;

        private State(double motorVoltage) {
            this.motorVoltage = motorVoltage;
        }

        public double getMotorVoltage() {
            return motorVoltage;
        }
    }

    public CommandBase setRollerStateCommand(State state) {
        return new InstantCommand(() -> setRollerState(state));
    }

    public Command waitForGamePiece() {
        return new WaitUntilCommand(() -> m_inputs.rollerCurrentAmps[0] > 40.0);
    }

    public Command intakeConeCommand() {
        return Commands
                .sequence(setRollerStateCommand(State.INTAKING_CONE), new WaitCommand(0.5),
                        waitForGamePiece())
                .finallyDo(interupted -> setRollerState(State.HOLD_CONE));
    }

    public Command intakeCubeCommand() {
        return Commands
                .sequence(setRollerStateCommand(State.INTAKING_CUBE), new WaitCommand(0.5),
                        waitForGamePiece())
                .finallyDo(interupted -> setRollerStateCommand(State.HOLD_CUBE));
    }

    public Command idleCommand() {
        return setRollerStateCommand(State.IDLE);
    }

    public Command scoreCone() {
        return Commands.sequence(setRollerStateCommand(State.EJECT_CONE), new WaitCommand(0.5),
                setRollerStateCommand(State.IDLE));
    }

    public Command scoreCube() {
        return Commands.sequence(setRollerStateCommand(State.EJECT_CUBE), new WaitCommand(0.5),
                setRollerStateCommand(State.IDLE));
    }

}
