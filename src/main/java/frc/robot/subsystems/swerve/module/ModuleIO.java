package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {

    public static class ModuleIOInputs implements LoggableInputs {
        double driveDistanceMeters = 0.0;
        double driveVelocityMetersPerSec = 0.0;
        double driveAppliedVolts = 0.0;
        double[] driveCurrentAmps = new double[] {};
        double[] driveTempCelsius = new double[] {};

        double angleAbsolutePositionRad = 0.0;
        double angleInternalPositionRad = 0.0;
        double angleInternalVelocityRadPerSec = 0.0;
        double angleAppliedVolts = 0.0;
        double[] angleCurrentAmps = new double[] {};
        double[] angleTempCelsius = new double[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("DriveDistanceMeters", driveDistanceMeters);
            table.put("DriveVelocityMetersPerSec", driveVelocityMetersPerSec);
            table.put("DriveAppliedVolts", driveAppliedVolts);
            table.put("DriveCurrentAmps", driveCurrentAmps);
            table.put("DriveTempCelsius", driveTempCelsius);

            table.put("AngleAbsolutePositionRad", angleAbsolutePositionRad);
            table.put("AngleInternalPositionRad", angleInternalPositionRad);
            table.put("AngleInternalVelocityRadPerSec", angleInternalVelocityRadPerSec);
            table.put("AngleAppliedVolts", angleAppliedVolts);
            table.put("AngleCurrentAmps", angleCurrentAmps);
            table.put("AngleTempCelsius", angleTempCelsius);
        }

        @Override
        public void fromLog(LogTable table) {

        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified percentage of full power. */
    public default void setDriveMotorPercentage(double percentage) {}

    /** Run the drive motor at the specified voltage. */
    public default void setDriveVoltage(double voltage) {}

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocity) {}

    /** Run the drive motor at the specified voltage. */
    public default void setAngleVoltage(double voltage) {}

    /** Run the turn motor to the specified angle. */
    public default void setAnglePosition(double radians) {}

    /** Enable or disable brake mode on the drive motor. */
    public default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable brake mode on the turn motor. */
    public default void setAngleBrakeMode(boolean enable) {}

    public default boolean resetToAbsolute() {
        return false;
    }

}
