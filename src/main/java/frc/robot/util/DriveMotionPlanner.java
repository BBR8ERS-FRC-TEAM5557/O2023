package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.lib.team6328.TunableNumber;
import frc.lib.team6328.VirtualSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class DriveMotionPlanner extends VirtualSubsystem {
    private static final Swerve swerve = RobotContainer.m_swerve;

    private static final PIDController forwardController =
            new PIDController(kTranslationkP, kTranslationkI, kTranslationkD);
    private static final PIDController strafeController =
            new PIDController(kTranslationkP, kTranslationkI, kTranslationkD);
    private static final PIDController rotationController =
            new PIDController(kRotationkP, kRotationkI, kRotationkD);
    private static final ProfiledPIDController snapController = new ProfiledPIDController(
            kRotationkP, kRotationkI, kRotationkD, new Constraints(kSnapMaxOmega, kSnapMaxAlpha));

    private static final TunableNumber translationkP =
            new TunableNumber("Swerve/TranslationKP", kTranslationkP);
    private static final TunableNumber translationkI =
            new TunableNumber("Swerve/TranslationKI", kTranslationkI);
    private static final TunableNumber translationkD =
            new TunableNumber("Swerve/TranslationKD", kTranslationkD);

    private static final TunableNumber rotationkP =
            new TunableNumber("Swerve/RotationKP", kRotationkP);
    private static final TunableNumber rotationkI =
            new TunableNumber("Swerve/RotationKI", kRotationkI);
    private static final TunableNumber rotationkD =
            new TunableNumber("Swerve/RotationKD", kRotationkD);

    private static final TunableNumber snapMaxOmega =
            new TunableNumber("Swerve/SnapMaxOmega", kSnapMaxOmega);
    private static final TunableNumber snapMaxAlpha =
            new TunableNumber("Swerve/SnapMaxAlpha", kSnapMaxAlpha);

    public static void checkForUpdates() {
        if (translationkP.hasChanged(translationkP.hashCode())
                || translationkI.hasChanged(translationkI.hashCode())
                || translationkD.hasChanged(translationkD.hashCode())
                || rotationkP.hasChanged(rotationkP.hashCode())
                || rotationkI.hasChanged(rotationkI.hashCode())
                || rotationkD.hasChanged(rotationkD.hashCode())
                || snapMaxOmega.hasChanged(snapMaxOmega.hashCode())
                || snapMaxAlpha.hasChanged(snapMaxAlpha.hashCode())) {
            forwardController.setPID(translationkP.get(), translationkI.get(), translationkD.get());
            strafeController.setPID(translationkP.get(), translationkI.get(), translationkD.get());
            rotationController.setPID(rotationkP.get(), rotationkI.get(), rotationkD.get());
            snapController.setPID(rotationkP.get(), rotationkI.get(), rotationkD.get());
            snapController.setConstraints(new Constraints(snapMaxOmega.get(), snapMaxAlpha.get()));

        }
    }

    public static void configureControllers() {
        rotationController.enableContinuousInput(0, Math.PI * 2.0);
        snapController.enableContinuousInput(0, Math.PI * 2.0);
    }

    @Override
    public void periodic() {
        checkForUpdates();
    }
    
    public static double calculateSnap(Rotation2d goalAngle) {
        return snapController.calculate(swerve.getYaw().getRadians(), goalAngle.getRadians());
    }

    public static PIDController getForwardController() {
        return forwardController;
    }

    public static PIDController getStrafeController() {
        return strafeController;
    }

    public static PIDController getRotationController() {
        return rotationController;
    }

    public static ProfiledPIDController getSnapController() {
        return snapController;
    }

}
