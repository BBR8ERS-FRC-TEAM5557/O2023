package frc.robot.subsystems.superstructure;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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

        public static SuperstructureGoal currentGoal = SuperstructureGoal.STOW;

        public static SuperstructureGoal getCurrentGoal() {
                return currentGoal;
        }

        public static void setCurrentGoal(SuperstructureGoal cur) {
                currentGoal = cur;
        }

        public static Command setSuperstructureGoal(SuperstructureGoal goal) {
                return Commands.sequence(
                                new InstantCommand(() -> setCurrentGoal(goal)),
                                elevator.setElevatorHeightProfiled(goal.elevator),
                                elevator.extendWaitCommand(goal.elevator - 15.0),
                                wrist.setWristAngleProfiled(goal.wrist));
        }

        public static Command epsilonWaitCommand() {
                return Commands.sequence(elevator.epsilonWaitCommand(), wrist.epsilonWaitCommand());
        }

        public static Command setSuperstructureScore(Supplier<NodeLevel> level,
                        Supplier<GamePiece> piece) {
                return Commands.either(
                                Commands.select(
                                                Map.of(NodeLevel.HYBRID,
                                                                setSuperstructureGoal(SuperstructureGoal.L1_SCORE),
                                                                NodeLevel.MID,
                                                                setSuperstructureGoal(SuperstructureGoal.L2_CONE),
                                                                NodeLevel.HIGH,
                                                                setSuperstructureGoal(SuperstructureGoal.L3_CONE)),
                                                () -> level.get()),
                                Commands.select(
                                                Map.of(NodeLevel.HYBRID,
                                                                setSuperstructureGoal(SuperstructureGoal.L1_SCORE),
                                                                NodeLevel.MID,
                                                                setSuperstructureGoal(SuperstructureGoal.L2_CUBE),
                                                                NodeLevel.HIGH,
                                                                setSuperstructureGoal(SuperstructureGoal.L3_CUBE)),
                                                () -> level.get()),
                                () -> piece.get() == GamePiece.CONE);
        }

        public static Command setScoreTeleop() {
                return Commands.sequence(setSuperstructureScore(ObjectiveTracker::getNodeLevel,
                                ObjectiveTracker::getGamePiece), new WaitUntilCommand(() -> false))
                                .finallyDo(interrupted -> setSuperstructureGoal(SuperstructureGoal.STOW).schedule());
        }

        public static Command setStow() {
                return setSuperstructureGoal(SuperstructureGoal.STOW);
        }

        public static Command scoreCubeLevel(NodeLevel level) {
                return Commands.sequence(setSuperstructureScore(() -> level, () -> GamePiece.CUBE),
                                epsilonWaitCommand().withTimeout(3.0), roller.scoreCube(),
                                setSuperstructureGoal(SuperstructureGoal.STOW));
        }

        public static Command scoreConeLevel(NodeLevel level) {
                return Commands.sequence(setSuperstructureScore(() -> level, () -> GamePiece.CONE),
                                epsilonWaitCommand().withTimeout(3.0), roller.scoreCone(),
                                setSuperstructureGoal(SuperstructureGoal.STOW));
        }

        public static Command intakeGroundCone() {
                return Commands
                                .sequence(
                                        new InstantCommand(() -> ObjectiveTracker.setGamePiece(GamePiece.CONE)),
                                                Commands.deadline(roller.intakeConeCommand(),
                                                                setSuperstructureGoal(
                                                                                SuperstructureGoal.GROUND_CONE_INTAKE)),
                                                new WaitCommand(0.0))
                                .finallyDo(interrupted -> setSuperstructureGoal(SuperstructureGoal.STOW).schedule());
        }

        public static Command intakeGroundCube() {
                return Commands
                                .sequence(
                                        new InstantCommand(() -> ObjectiveTracker.setGamePiece(GamePiece.CUBE)),
                                                Commands.deadline(roller.intakeCubeCommand(),
                                                                setSuperstructureGoal(
                                                                                SuperstructureGoal.GROUND_CUBE_INTAKE)),
                                                new WaitCommand(0.0))
                                .finallyDo(interrupted -> setSuperstructureGoal(SuperstructureGoal.STOW).schedule());
        }

        public static Command intakeSubstation() {
                return Commands
                                .sequence(
                                        new InstantCommand(() -> ObjectiveTracker.setGamePiece(GamePiece.CONE)),
                                                Commands.deadline(roller.intakeConeCommand(),
                                                                setSuperstructureGoal(
                                                                                SuperstructureGoal.SHELF_CONE_INTAKE)),
                                                new WaitCommand(0.0))
                                .finallyDo(interrupted -> setSuperstructureGoal(SuperstructureGoal.STOW).schedule());
        }

        public static enum SuperstructureGoal {
                STOW(1.5, 260.0),

                GROUND_CONE_INTAKE(0.0, 190.0), GROUND_CUBE_INTAKE(0.0, 180.0),

                SHELF_CONE_INTAKE(5.0, 200.0), SHELF_CUBE_INTAKE(5.0, 200.0),

                SLIDE_CUBE_INTAKE(5.0, 200.0),

                SCORE_STANDBY(5.0, 200.0),

                L1_SCORE(0.0, 235.0),

                L2_CONE(15.8, 210.0), L2_CUBE(12.0, 200.0),

                L3_CONE(23.0, 210.0), L3_CUBE(20.0, 200.0);

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
