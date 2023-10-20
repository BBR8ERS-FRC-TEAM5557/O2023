package frc.robot.auto;

import java.io.File;
import java.io.FileFilter;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.ObjectiveTracker.GamePiece;
import frc.robot.subsystems.superstructure.ObjectiveTracker.NodeLevel;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.commands.AutoBalance;
import frc.robot.util.DriveMotionPlanner;
import frc.robot.util.RobotStateEstimator;

public class AutoRoutineManager {
    private final LoggedDashboardChooser<Command> m_chooser;
    private final HashMap<String, Command> m_eventMap;
    private final HashMap<String, PathPlannerTrajectory> m_trajectoryMap;

    private final Swerve swerve;
    private final Elevator elevator;

    public AutoRoutineManager(Swerve swerve, Elevator elevator) {
        System.out.println("[Init] Creating Auto Routine Manager");
        m_chooser = new LoggedDashboardChooser<Command>("AutonomousChooser");
        m_eventMap = new HashMap<>();
        m_trajectoryMap = new HashMap<>();

        this.swerve = swerve;
        this.elevator = elevator;

        generateEventMap();
        generateTrajectories();
        generateAutoChoices();
        Logger.getInstance().recordOutput("AutoTraj", new Trajectory());

        if (Constants.kTuningMode) {
            PathPlannerServer.startServer(5811);
        }
    }

    private void generateAutoChoices() {
        m_chooser.addDefaultOption("Do Nothing", null);

        m_chooser.addOption("3 Piece (F)", Commands.sequence(new PrintCommand("Starting 3 piece"),
                getPoseResetCommand(m_trajectoryMap.get("LeftToSweepToCubeScore")),
                Superstructure.scoreConeLevel(NodeLevel.HIGH), elevator.tuckWaitCommand(10.0),
                getFollowComand(m_trajectoryMap.get("LeftToSweepToCubeScore")),
                Superstructure.scoreCubeLevel(NodeLevel.HIGH), elevator.tuckWaitCommand(10.0),
                getFollowComand(m_trajectoryMap.get("LeftCubeScoreToLeftMidIntake")),
                getFollowComand(m_trajectoryMap.get("LeftMidIntakeToConeScore")),
                getFollowComand(m_trajectoryMap.get("ConeScoreToEvacLeft"))));

        m_chooser.addOption("2 Piece + 1 + Balance (F)", Commands.sequence(
                new PrintCommand("Starting 2 piece w/ balance"),
                getPoseResetCommand(m_trajectoryMap.get("LeftToSweepToCubeScore")),
                Superstructure.scoreConeLevel(NodeLevel.HIGH), elevator.tuckWaitCommand(10.0),
                getFollowComand(m_trajectoryMap.get("LeftToSweepToCubeScore")),
                Superstructure.scoreCubeLevel(NodeLevel.HIGH), elevator.tuckWaitCommand(10.0),
                getFollowComand(m_trajectoryMap.get("LeftCubeScoreToLeftMidIntake")),
                getFollowComand(m_trajectoryMap.get("LeftMidIntakeToBalance")),
                AutoBalance.autoBalanceCommand()));

        m_chooser.addOption("2 Piece (F)",
                Commands.sequence(new PrintCommand("2 Piece (F)"),
                        getPoseResetCommand(m_trajectoryMap.get("LeftToSweepToCubeScore")),
                        Superstructure.scoreConeLevel(NodeLevel.HIGH),
                        elevator.tuckWaitCommand(10.0),
                        getFollowComand(m_trajectoryMap.get("LeftToSweepToCubeScore")),
                        Superstructure.scoreCubeLevel(NodeLevel.HIGH)));

        m_chooser.addOption("1 Piece + Evac/Intake (F)",
                Commands.sequence(new PrintCommand("Starting 1 Piece + 1 Pause (F)"),
                        getPoseResetCommand(m_trajectoryMap.get("LeftToSweepPause")),
                        Superstructure.scoreConeLevel(NodeLevel.HIGH),
                        elevator.tuckWaitCommand(10.0),
                        getFollowComand(m_trajectoryMap.get("LeftToSweepPause"))));

        m_chooser.addOption("Score Cone + Balance (M)", Commands.sequence(
                new PrintCommand("Starting Score Cone + Balance"),
                getPoseResetCommand(m_trajectoryMap.get("MiddleToBalance")),
                Superstructure.scoreConeLevel(NodeLevel.HIGH), elevator.tuckWaitCommand(5.0),
                getFollowComand(m_trajectoryMap.get("MiddleToBalance")),
                AutoBalance.autoBalanceCommand()));

        m_chooser.addOption("1 Piece + Evac/Intake (B)",
                Commands.sequence(new PrintCommand("Starting 1 Piece + 1 Pause (B)"),
                        getPoseResetCommand(m_trajectoryMap.get("RightToIntake")),
                        Superstructure.scoreConeLevel(NodeLevel.HIGH),
                        elevator.tuckWaitCommand(10.0),
                        getFollowComand(m_trajectoryMap.get("RightToIntake"))));

        m_chooser.addOption("Score Cone (A)",
                Commands.sequence(new PrintCommand("Starting Score Cone"),
                        getPoseResetCommand(m_trajectoryMap.get("RightToIntake")),
                        Superstructure.scoreConeLevel(NodeLevel.HIGH)));

    }

    private void generateTrajectories() {
        File dir = new File(Filesystem.getDeployDirectory() + "/pathplanner");
        FileFilter filter = new FileFilter() {
            @Override
            public boolean accept(File arg0) {
                return !arg0.isDirectory() && arg0.getName().endsWith(".path");
            }
        };

        File[] files = dir.listFiles(filter);
        for (File file : files) {
            int index = file.getName().lastIndexOf('.');
            String prefix = file.getName().substring(0, index);
            m_trajectoryMap.put(prefix,
                    PathPlanner.loadPath(prefix, SwerveConstants.kMaxAttainableSpeed,
                            SwerveConstants.kMaxAttainableAcceleration));
            System.out.println("Generated trajectory: " + prefix);
        }
    }

    private void generateEventMap() {
        m_eventMap.put("intakeCube", Superstructure.intakeGroundCube().withTimeout(3.0));
        m_eventMap.put("intakeCone", new PrintCommand("[Intaking Cone!!!]")
                .andThen(Superstructure.intakeGroundCone().withTimeout(3.0)));

        m_eventMap.put("prepScoreHighCone", new PrintCommand("[Scoring High Cone!!!]").andThen(
                Superstructure.setSuperstructureScore(() -> NodeLevel.HIGH, () -> GamePiece.CONE)));
        m_eventMap.put("prepScoreHighCube", new PrintCommand("[Scoring High Cube!!!]").andThen(
                Superstructure.setSuperstructureScore(() -> NodeLevel.HIGH, () -> GamePiece.CUBE)));
    }

    private Command getFollowComand(PathPlannerTrajectory path) {
        PPSwerveControllerCommand followCommand = new PPSwerveControllerCommand(path,
                RobotStateEstimator.getInstance()::getPose,
                DriveMotionPlanner.getForwardController(), // X controller
                DriveMotionPlanner.getStrafeController(), // Y controller
                DriveMotionPlanner.getRotationController(), // Rotation controller
                swerve::driveOpenLoop, // Module states consumer
                true, // Should the path be mirrored depending on alliance color.
                swerve // Requires swerve subsystem
        );

        return new FollowPathWithEvents(followCommand, path.getMarkers(), m_eventMap).alongWith(
                new InstantCommand(
                        () -> Logger.getInstance().recordOutput("PathFollowing", path)),
                new InstantCommand(() -> RobotStateEstimator.getInstance()
                        .addFieldTrajectory("AutoTrajectory", path)));
    }

    private Command getPoseResetCommand(PathPlannerTrajectory path) {
        PathPlannerState transformedState = PathPlannerTrajectory
                .transformStateForAlliance(path.getInitialState(), DriverStation.getAlliance());
        Pose2d transformedPose = new Pose2d(transformedState.poseMeters.getTranslation(),
                transformedState.holonomicRotation);

        return new InstantCommand(() -> setPose(transformedPose));
    }

    public Command getAutoCommand() {
        return m_chooser.get();
    }

    private void setPose(Pose2d pose) {
        RobotStateEstimator.getInstance().setPose(pose);
    }

}
