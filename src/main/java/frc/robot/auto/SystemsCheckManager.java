package frc.robot.auto;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.ObjectiveTracker.NodeLevel;
import frc.robot.subsystems.swerve.Swerve;

public class SystemsCheckManager {
    private final LoggedDashboardChooser<Command> m_chooser;

    private final Swerve swerve;

    private static final double kModuleRequestTransitionTime = 0.5;
    private static final double kModuleRequestRunTime = 2.0;
    private static final double kAzimuthCheckVoltage = 4.0;
    private static final double kDriveSpeedMPS = 2.0;
    private static final SwerveModuleState kSetpointCheckForward =
            new SwerveModuleState(kDriveSpeedMPS, new Rotation2d());
    private static final SwerveModuleState kSetpointCheckBackward =
            new SwerveModuleState(-kDriveSpeedMPS, new Rotation2d());

    public SystemsCheckManager(Swerve swerve) {
        System.out.println("[Init] Creating System Check Manager");
        m_chooser = new LoggedDashboardChooser<Command>("SystemsCheckChooser");

        this.swerve = swerve;

        generateSystemCheckChoices();
    }

    private void generateSystemCheckChoices() {
        m_chooser.addDefaultOption("--No Check Selected--", null);

        m_chooser.addOption("Drivetrain Check",
                Commands.sequence(runModuleCheckSequence(0), new WaitCommand(1.0),
                        runModuleCheckSequence(1), new WaitCommand(1.0), runModuleCheckSequence(2),
                        new WaitCommand(1.0), runModuleCheckSequence(3)));

        m_chooser.addOption("Superstructure Cone Check",
                Commands.sequence(
                        Superstructure.intakeGroundCone(), 
                        new WaitCommand(1.0),
                        Superstructure.scoreConeLevel(NodeLevel.HYBRID),
                        Superstructure.epsilonWaitCommand(),
                        new WaitCommand(1.0),

                        Superstructure.intakeGroundCone(), new WaitCommand(1.0),
                        Superstructure.scoreConeLevel(NodeLevel.MID),
                        Superstructure.epsilonWaitCommand(), new WaitCommand(1.0),

                        Superstructure.intakeGroundCone(), new WaitCommand(1.0),
                        Superstructure.scoreCubeLevel(NodeLevel.HIGH),
                        Superstructure.epsilonWaitCommand()));
    }

    public Command getCheckCommand() {
        return m_chooser.get();
    }

    public Command runModuleCheckSequence(int modNumber) {
        return Commands.sequence(new PrintCommand("Running Mod. " + modNumber + " system check"),
                new RunCommand(() -> swerve.runModuleCheck(modNumber,
                        SwerveModuleSystemCheckRequest.AZIMUTH_FORWARD), swerve)
                                .withTimeout(kModuleRequestRunTime),
                new RunCommand(() -> swerve.runModuleCheck(modNumber,
                        SwerveModuleSystemCheckRequest.DO_NOTHING), swerve)
                                .withTimeout(kModuleRequestTransitionTime),

                new RunCommand(() -> swerve.runModuleCheck(modNumber,
                        SwerveModuleSystemCheckRequest.AZIMUTH_BACKWARD), swerve)
                                .withTimeout(kModuleRequestRunTime),
                new RunCommand(() -> swerve.runModuleCheck(modNumber,
                        SwerveModuleSystemCheckRequest.DO_NOTHING), swerve)
                                .withTimeout(kModuleRequestTransitionTime),

                new RunCommand(() -> swerve.runModuleCheck(modNumber,
                        SwerveModuleSystemCheckRequest.SETPOINT_FORWARD), swerve)
                                .withTimeout(kModuleRequestRunTime),
                new RunCommand(() -> swerve.runModuleCheck(modNumber,
                        SwerveModuleSystemCheckRequest.DO_NOTHING), swerve)
                                .withTimeout(kModuleRequestTransitionTime),

                new RunCommand(() -> swerve.runModuleCheck(modNumber,
                        SwerveModuleSystemCheckRequest.SETPOINT_BACKWARD), swerve)
                                .withTimeout(kModuleRequestRunTime),
                new RunCommand(() -> swerve.runModuleCheck(modNumber,
                        SwerveModuleSystemCheckRequest.DO_NOTHING), swerve)
                                .withTimeout(kModuleRequestTransitionTime));
    }

    public enum SwerveModuleSystemCheckRequest {
        DO_NOTHING(0.0), AZIMUTH_FORWARD(kAzimuthCheckVoltage), AZIMUTH_BACKWARD(
                -kAzimuthCheckVoltage), SETPOINT_FORWARD(
                        kSetpointCheckForward), SETPOINT_BACKWARD(kSetpointCheckBackward);

        private double steerVoltage = Double.NaN;
        private SwerveModuleState setpointState = null;

        private SwerveModuleSystemCheckRequest(double steerVoltage,
                SwerveModuleState setpointState) {
            this.steerVoltage = steerVoltage;
            this.setpointState = setpointState;
        }

        private SwerveModuleSystemCheckRequest(SwerveModuleState state) {
            this(0.0, state);
        }

        private SwerveModuleSystemCheckRequest(double steerVoltage) {
            this(steerVoltage, null);
        }

        public boolean isSetpointCheck() {
            return setpointState != null;
        }

        public SwerveModuleState getSwerveModuleState() {
            return this.setpointState;
        }

        public double getSteerVoltage() {
            return this.steerVoltage;
        }
    }
}
