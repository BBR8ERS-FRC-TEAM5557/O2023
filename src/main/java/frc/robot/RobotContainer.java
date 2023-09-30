// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoRoutineManager;
import frc.robot.auto.SystemsCheckManager;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOSparkMax;
import frc.robot.subsystems.superstructure.ObjectiveTracker;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.ObjectiveTracker.Direction;
import frc.robot.subsystems.superstructure.ObjectiveTracker.GamePiece;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.commands.TeleopDrive;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOSparkMax;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSparkMax;
import frc.robot.util.DriveMotionPlanner;
import frc.robot.util.RobotStateEstimator;
import frc.robot.util.Util;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotMap.*;

public class RobotContainer {

    public static final XboxController m_driver = new XboxController(0);
    public static final XboxController m_operator = new XboxController(1);
    public static Swerve m_swerve;
    public static Elevator m_elevator;
    public static Wrist m_wrist;
    public static Roller m_roller;

    public static RobotStateEstimator m_stateEstimator;

    public static AutoRoutineManager m_autoManager;
    public static SystemsCheckManager m_systemCheckManager;

    public RobotContainer() {
        if (kIsReal) {
            m_swerve = new Swerve(new GyroIOPigeon2(),
                    new ModuleIOSparkMax(0, kFLDriveMotor, kFLTurnMotor, kFLCancoder, kFLOffset),
                    new ModuleIOSparkMax(1, kFRDriveMotor, kFRTurnMotor, kFRCancoder, kFROffset),
                    new ModuleIOSparkMax(2, kBLDriveMotor, kBLTurnMotor, kBLCancoder, kBLOffset),
                    new ModuleIOSparkMax(3, kBRDriveMotor, kBRTurnMotor, kBRCancoder, kBROffset));
            m_elevator = new Elevator(new ElevatorIOSparkMax());
            m_wrist = new Wrist(new WristIOSparkMax());
            m_roller = new Roller(new RollerIOSparkMax());
        } else {
            m_swerve = new Swerve(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(),
                    new ModuleIOSim(), new ModuleIOSim());
        }

        // Instantiate missing subsystems
        if (m_swerve == null) {
            m_swerve = new Swerve(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {},
                    new ModuleIO() {}, new ModuleIO() {});
        }
        if (m_elevator == null) {
            m_elevator = new Elevator(new ElevatorIO() {});
        }
        if (m_wrist == null) {
            m_wrist = new Wrist(new WristIO() {});
        }
        if (m_roller == null) {
            m_roller = new Roller(new RollerIO() {});
        }

        m_autoManager = new AutoRoutineManager(m_swerve, m_elevator);
        m_systemCheckManager = new SystemsCheckManager(m_swerve);
        m_stateEstimator = RobotStateEstimator.getInstance();
        DriveMotionPlanner.configureControllers();

        configureBindings();

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Driver");
        shuffleboardTab.addString("Node Type", () -> ObjectiveTracker.getNodeLevel().name() + " "
                + ObjectiveTracker.getGamePiece().name());
        shuffleboardTab.addString("Super State", () -> Superstructure.getCurrentGoal().name());
    }

    private void configureBindings() {
        // Bind driver and operator controls
        System.out.println("[Init] Binding controls");
        DriverStation.silenceJoystickConnectionWarning(true);

        m_swerve.setDefaultCommand(new TeleopDrive(this::getForwardInput, this::getStrafeInput,
                this::getRotationInput, m_driver::getRightBumper));

        // Reset swerve heading
        new Trigger(m_driver::getStartButton)
                .onTrue(new InstantCommand(() -> m_stateEstimator.setPose(new Pose2d())));

        // Driver sets cone intake
        new Trigger(m_driver::getLeftBumper)
                .onTrue(new InstantCommand(
                        () -> m_swerve.setKinematicLimits(SwerveConstants.kScoringLimits)))
                .onFalse(new InstantCommand(
                        () -> m_swerve.setKinematicLimits(SwerveConstants.kUncappedLimits)))
                .whileTrue(Superstructure.intakeGroundCone());

        // Driver sets cube intake
        new Trigger(() -> m_driver.getLeftTriggerAxis() > 0.5)
                .onTrue(new InstantCommand(
                        () -> m_swerve.setKinematicLimits(SwerveConstants.kScoringLimits)))
                .onFalse(new InstantCommand(
                        () -> m_swerve.setKinematicLimits(SwerveConstants.kUncappedLimits)))
                .whileTrue(Superstructure.intakeGroundCube());

        // Driver sets Substation intake
        new Trigger(() -> m_driver.getRightTriggerAxis() > 0.5)
                .onTrue(new InstantCommand(
                        () -> m_swerve.setKinematicLimits(SwerveConstants.kScoringLimits)))
                .onFalse(new InstantCommand(
                        () -> m_swerve.setKinematicLimits(SwerveConstants.kUncappedLimits)))
                .whileTrue(Superstructure.intakeSubstation());

        // Sets superstructure state on operator Right Trigger hold
        new Trigger(() -> m_operator.getRightTriggerAxis() > 0.5)
                .onTrue(new InstantCommand(
                        () -> m_swerve.setKinematicLimits(SwerveConstants.kScoringLimits)))
                .onFalse(new InstantCommand(
                        () -> m_swerve.setKinematicLimits(SwerveConstants.kUncappedLimits)))
                .whileTrue(Commands.sequence(new WaitUntilCommand(
                        () -> m_swerve.isUnderKinematicLimit(SwerveConstants.kScoringLimits)),
                        Superstructure.setScoreTeleop()))
                .onFalse(Superstructure.setStow());

        // Ejects gamepiece when operator presses A button
        new Trigger(m_operator::getAButton).onTrue(new ConditionalCommand(m_roller.scoreCone(),
                m_roller.scoreCube(), () -> ObjectiveTracker.getGamePiece() == GamePiece.CONE));

        // Adjusts the scoring objective
        new Trigger(() -> m_operator.getPOV() == 0)
                .onTrue(ObjectiveTracker.shiftNodeCommand(Direction.UP));
        new Trigger(() -> m_operator.getPOV() == 180)
                .onTrue(ObjectiveTracker.shiftNodeCommand(Direction.DOWN));

        // Manual Elevator
        new Trigger(m_operator::getLeftStickButton).whileTrue(m_elevator.homeElevator());

        new Trigger(() -> m_operator.getRightBumper() && m_operator.getLeftBumper())
                .whileTrue(m_elevator.runElevatorOpenLoop(() -> getElevatorJogger()))
                .whileTrue(m_wrist.runWristOpenLoop(() -> getWristJogger()))
                .onFalse(m_elevator.runElevatorOpenLoop(() -> 0.0))
                .onFalse(m_wrist.runWristOpenLoop(() -> 0.0));

        // Manual Wrist
        new Trigger(m_operator::getRightStickButton).whileTrue(m_wrist.homeWrist());

        // Endgame alerts
        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0.0
                && DriverStation.getMatchTime() <= Math.round(30.0)).onTrue(Commands.run(() -> {
                    LEDs.getInstance().endgameAlert = true;
                    m_driver.setRumble(RumbleType.kBothRumble, 1.0);
                    m_operator.setRumble(RumbleType.kBothRumble, 1.0);
                }).withTimeout(1.5).andThen(Commands.run(() -> {
                    LEDs.getInstance().endgameAlert = false;
                    m_driver.setRumble(RumbleType.kBothRumble, 0.0);
                    m_operator.setRumble(RumbleType.kBothRumble, 0.0);
                }).withTimeout(1.0)));

        new Trigger(() -> DriverStation.isTeleopEnabled() && DriverStation.getMatchTime() > 0.0
                && DriverStation.getMatchTime() <= Math.round(15.0))
                        .onTrue(Commands.sequence(Commands.run(() -> {
                            LEDs.getInstance().endgameAlert = true;
                            m_driver.setRumble(RumbleType.kBothRumble, 1.0);
                            m_operator.setRumble(RumbleType.kBothRumble, 1.0);
                        }).withTimeout(0.5), Commands.run(() -> {
                            LEDs.getInstance().endgameAlert = false;
                            m_driver.setRumble(RumbleType.kBothRumble, 0.0);
                            m_operator.setRumble(RumbleType.kBothRumble, 0.0);
                        }).withTimeout(0.5), Commands.run(() -> {
                            LEDs.getInstance().endgameAlert = true;
                            m_driver.setRumble(RumbleType.kBothRumble, 1.0);
                            m_operator.setRumble(RumbleType.kBothRumble, 1.0);
                        }).withTimeout(0.5), Commands.run(() -> {
                            LEDs.getInstance().endgameAlert = false;
                            m_driver.setRumble(RumbleType.kBothRumble, 0.0);
                            m_operator.setRumble(RumbleType.kBothRumble, 0.0);
                        }).withTimeout(1.0)));

    }

    public Command getAutonomousCommand() {
        return m_autoManager.getAutoCommand();
    }

    public Command getSubsystemCheckCommand() {
        return m_systemCheckManager.getCheckCommand();
    }

    public double getForwardInput() {
        return -square(deadband(m_driver.getLeftY(), 0.15));
    }

    public double getStrafeInput() {
        return -square(deadband(m_driver.getLeftX(), 0.15));
    }

    public double getRotationInput() {
        return -square(deadband(m_driver.getRightX(), 0.15));
    }

    public double getElevatorJogger() {
        return -square(deadband(m_operator.getLeftY(), 0.15));
    }

    public double getWristJogger() {
        return -square(deadband(m_operator.getRightY(), 0.15));
    }

    private static double deadband(double value, double tolerance) {
        if (Math.abs(value) < tolerance)
            return 0.0;

        return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    public static double square(double value) {
        return Math.copySign(value * value, value);
    }
}
