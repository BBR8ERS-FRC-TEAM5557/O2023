package frc.robot.util;

import java.util.List;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.team6328.VirtualSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

public class RobotStateEstimator extends VirtualSubsystem {

    public static RobotStateEstimator m_instance = null;

    private final Swerve m_swerve = RobotContainer.m_swerve;
    private final Field2d m_field2d = new Field2d();
    private final Pose2d[] modulePoses = new Pose2d[4];

    public static RobotStateEstimator getInstance() {
        if (m_instance == null) {
            System.out.println("[Init] Creating RobotStateEstimator");
            m_instance = new RobotStateEstimator();
        }
        return m_instance;
    }

    private SwerveDrivePoseEstimator m_poseEstimator;
    private SwerveModulePosition[] m_lastModulePositions = new SwerveModulePosition[] { new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition() };

    private RobotStateEstimator() {
        m_poseEstimator = new SwerveDrivePoseEstimator(Swerve.m_kinematics, new Rotation2d(),
                m_lastModulePositions, new Pose2d());

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Driver");
        shuffleboardTab.add(m_field2d);
    }

    @Override
    public void periodic() {
        clampPoseToField();
        updateFieldWidget();

        Logger.getInstance().recordOutput("Odometry/Robot", getPose());
    }

    /** Records a new drive movement without gyro. */
    public void addDriveData(SwerveModulePosition[] positions) {
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            wheelDeltas[i] = new SwerveModulePosition(
                    (positions[i].distanceMeters - m_lastModulePositions[i].distanceMeters),
                    positions[i].angle);
            m_lastModulePositions[i] = positions[i];
        }
        var twist = Swerve.m_kinematics.toTwist2d(wheelDeltas);
        var simulatedGyroAngle = Rotation2d.fromRadians(getPose().getRotation().getRadians() + twist.dtheta);

        addDriveData(simulatedGyroAngle, positions);
    }

    /** Records a new drive movement with gyro. */
    public void addDriveData(Rotation2d gyroAngle, SwerveModulePosition[] positions) {
        for (int i = 0; i < 4; i++) {
            m_lastModulePositions[i] = positions[i];
        }
        m_poseEstimator.updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, positions);
    }

    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
        for (var update : visionData) {
            m_poseEstimator.addVisionMeasurement(update.pose(), update.timestamp(), update.stdDevs());
        }
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        m_poseEstimator.resetPosition(
                RobotContainer.m_swerve.getRawGyroYaw().orElse(pose.getRotation()),
                m_lastModulePositions, pose);
    }

    private void clampPoseToField() {
        // if out of bounds, clamp to field
        double estimatedXPos = m_poseEstimator.getEstimatedPosition().getX();
        double estimatedYPos = m_poseEstimator.getEstimatedPosition().getY();
        if (estimatedYPos < 0.0 || estimatedYPos > 8.35 || estimatedXPos < 0.0
                || estimatedXPos > Units.feetToMeters(52)) {
            double clampedYPosition = MathUtil.clamp(estimatedYPos, 0.0, 8.35);
            double clampedXPosition = MathUtil.clamp(estimatedXPos, 0.0, Units.feetToMeters(52.0));
            this.setPose(new Pose2d(clampedXPosition, clampedYPosition,
                    getPose().getRotation()));
        }
    }

    private void updateFieldWidget() {
        SwerveModuleState[] moduleStates = m_swerve.getSwerveSetpoint().moduleStates;
        Pose2d robotPose = getPose();

        for (int i = 0; i < modulePoses.length; i++) {
            Translation2d updatedPosition = SwerveConstants.kSwerveModuleLocations[i]
                    .rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
            Rotation2d updatedRotation = moduleStates[i].angle.plus(robotPose.getRotation());
            modulePoses[i] = new Pose2d(updatedPosition, updatedRotation);
        }

        m_field2d.setRobotPose(robotPose);
        addFieldPose("Swerve Modules", modulePoses);
    }

    public void addFieldPose(String name, Pose2d pose) {
        if (pose != null) {
            m_field2d.getObject(name).setPose(pose);
        }
    }

    private void addFieldPose(String name, Pose2d... pose) {
        if (pose != null) {
            m_field2d.getObject(name).setPoses(pose);
        }
    }

    public void addFieldTrajectory(String name, Trajectory traj) {
        if (traj != null) {
            m_field2d.getObject(name).setTrajectory(traj);
        }
    }

    /**
     * Represents a single vision pose with a timestamp and associated standard
     * deviations.
     */
    public static class TimestampedVisionUpdate {
        private final double timestamp;
        private final Pose2d pose;
        private final Matrix<N3, N1> stdDevs;

        public TimestampedVisionUpdate(double timestamp, Pose2d pose, Matrix<N3, N1> stdDevs) {
            this.timestamp = timestamp;
            this.pose = pose;
            this.stdDevs = stdDevs;
        }

        public double timestamp() {
            return timestamp;
        }

        public Pose2d pose() {
            return pose;
        }

        public Matrix<N3, N1> stdDevs() {
            return stdDevs;
        }
    }

}
