package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.DriveMotionPlanner;
import frc.robot.util.Util;

public class TeleopDrive extends CommandBase {
    private final Swerve swerve;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private final BooleanSupplier m_scoreLockSupplier;

    public TeleopDrive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier, BooleanSupplier scoreLockSupplier) {
        this.swerve = RobotContainer.m_swerve;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_scoreLockSupplier = scoreLockSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        // Go to X right before the end of the match
        if (DriverStation.getMatchTime() >= 0.0 && DriverStation.getMatchTime() < 0.25) {
            swerve.stopWithX();
            return;
        }

        var driveRotation = swerve.getYaw();

        var rotationalVelocity = m_rotationSupplier.getAsDouble() * swerve.getKinematicLimit().kMaxAngularVelocity;
        if (m_scoreLockSupplier.getAsBoolean()) {
            rotationalVelocity = DriveMotionPlanner.calculateSnap(Rotation2d.fromDegrees(180.0));
        }

        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                m_translationXSupplier.getAsDouble() * swerve.getKinematicLimit().kMaxDriveVelocity,
                m_translationYSupplier.getAsDouble() * swerve.getKinematicLimit().kMaxDriveVelocity,
                Util.clamp(rotationalVelocity, -swerve.getKinematicLimit().kMaxAngularVelocity,
                        swerve.getKinematicLimit().kMaxAngularVelocity),
                driveRotation);

        swerve.driveOpenLoop(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}
