package frc.robot.subsystems.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.superstructure.ObjectiveTracker.GamePiece;
import frc.robot.subsystems.superstructure.ObjectiveTracker.NodeLevel;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure {
    private static final Elevator elevator = RobotContainer.m_elevator;
    private static final Wrist wrist = RobotContainer.m_wrist;
    private static final Roller roller = RobotContainer.m_roller;

    public static SuperstructureGoal superGoal = SuperstructureGoal.STOW;

    public static Command setSuperstructureGoal(Supplier<SuperstructureGoal> goal) {
        superGoal = goal.get();
        return Commands.sequence(elevator.setElevatorHeightProfiled(goal.get().elevator),
                // elevator.extendWaitCommand(goal.get().elevator - 7.0),
                wrist.setWristAngleProfiled(goal.get().wrist));
    }

    public static Command epsilonWaitCommand() {
        return Commands.sequence(elevator.epsilonWaitCommand());
    }

    public static Command setSuperstructureScore(Supplier<NodeLevel> level,
            Supplier<GamePiece> piece) {
        SuperstructureGoal goal = SuperstructureGoal.L1_SCORE;
        if (piece.get() == GamePiece.CUBE) {
            switch (level.get()) {
                case HYBRID:
                    goal = SuperstructureGoal.L1_SCORE;
                    break;
                case MID:
                    goal = SuperstructureGoal.L2_CUBE;
                    break;
                case HIGH:
                    goal = SuperstructureGoal.L3_CUBE;
                    break;
            }
        } else {
            switch (level.get()) {
                case HYBRID:
                    goal = SuperstructureGoal.L1_SCORE;
                    break;
                case MID:
                    goal = SuperstructureGoal.L2_CONE;
                    break;
                case HIGH:
                    goal = SuperstructureGoal.L3_CONE;
                    break;
            }
        }
        final SuperstructureGoal finalGoal = goal;
        return setSuperstructureGoal(() -> finalGoal);
    }

    public static Command setScoreTeleop() {
        return setSuperstructureScore(ObjectiveTracker::getNodeLevel,
                ObjectiveTracker::getGamePiece);
    }

    public static Command setStow() {
        return setSuperstructureGoal(() -> SuperstructureGoal.STOW);
    }

    public static Command scoreCubeLevel(NodeLevel level) {
        return Commands.sequence(setSuperstructureScore(() -> level, () -> GamePiece.CUBE),
                epsilonWaitCommand().withTimeout(3.0), roller.scoreCube(),
                setSuperstructureGoal(() -> SuperstructureGoal.STOW));
    }

    public static Command scoreConeLevel(NodeLevel level) {
        return Commands.sequence(setSuperstructureScore(() -> level, () -> GamePiece.CONE),
                epsilonWaitCommand().withTimeout(3.0), roller.scoreCone(),
                setSuperstructureGoal(() -> SuperstructureGoal.STOW));
    }

    public static Command intakeGroundCone() {
        return Commands
                .sequence(
                        Commands.deadline(roller.intakeConeCommand(),
                                setSuperstructureGoal(() -> SuperstructureGoal.GROUND_CONE_INTAKE)),
                        new WaitCommand(0.5))
                .finallyDo(interupted -> setSuperstructureGoal(() -> SuperstructureGoal.STOW));
    }

    public static Command intakeGroundCube() {
        return Commands
                .sequence(
                        Commands.deadline(roller.intakeCubeCommand(),
                                setSuperstructureGoal(() -> SuperstructureGoal.GROUND_CUBE_INTAKE)),
                        new WaitCommand(0.5))
                .finallyDo(interupted -> setSuperstructureGoal(() -> SuperstructureGoal.STOW));
    }

    public static Command intakeSubstation() {
        return Commands
                .sequence(
                        Commands.deadline(roller.intakeConeCommand(),
                                setSuperstructureGoal(() -> SuperstructureGoal.SHELF_CONE_INTAKE)),
                        new WaitCommand(0.5))
                .finallyDo(interupted -> setSuperstructureGoal(() -> SuperstructureGoal.STOW));
    }


    public static enum SuperstructureGoal {
        STOW(0.0, 190.0),

        GROUND_CONE_INTAKE(5.0, 0.0), 
        GROUND_CUBE_INTAKE(5.0, 0.0),

        SHELF_CONE_INTAKE(5.0, 0.579607), 
        SHELF_CUBE_INTAKE(5.0, 0.579607),

        SLIDE_CUBE_INTAKE(5.0, 0.00),

        SCORE_STANDBY(5.0, 0.0),

        L1_SCORE(10.0, 110.0),

        L2_CONE(12.0, 0.556), 
        L2_CUBE(12.0, 0.556),

        L3_CONE(20.0, 1.04), 
        L3_CUBE(20.0, 1.04);

        public double elevator; // inches
        public double wrist; // degrees

        private SuperstructureGoal(double elevator, double wrist) {
            this.elevator = elevator;
            this.wrist = wrist;
        }

        // Default Constructor
        private SuperstructureGoal() {
            this(0, 0);
        }

    }
}
