package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import frc.robot.Robot;

public class ModuleIOSim implements ModuleIO {
    private FlywheelSim driveSim = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private FlywheelSim turnSim = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private final PIDController anglePID = new PIDController(23.0, 0.0, 0.0);
    //private final PIDController drivePID = new PIDController(0.0, 0.0, 0.0);

    private double turnAbsolutePositionRad = Math.random() * 2.0 * Math.PI;
    private double turnRelativePositionRad = turnAbsolutePositionRad;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public ModuleIOSim() {
        System.out.println("[Init] Creating ModuleIOSim");
        anglePID.enableContinuousInput(0, 2.0 * Math.PI);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Robot.defaultPeriodSecs);
        turnSim.update(Robot.defaultPeriodSecs);

        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * Robot.defaultPeriodSecs;
        turnRelativePositionRad += angleDiffRad;
        turnAbsolutePositionRad += angleDiffRad;
        while (turnAbsolutePositionRad < 0) {
            turnAbsolutePositionRad += 2.0 * Math.PI;
        }
        while (turnAbsolutePositionRad > 2.0 * Math.PI) {
            turnAbsolutePositionRad -= 2.0 * Math.PI;
        }

        inputs.driveDistanceMeters = inputs.driveDistanceMeters
                + (driveSim.getAngularVelocityRPM() * kWheelCircumference / 60.0) * Robot.defaultPeriodSecs;
        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRPM() * kWheelCircumference / 60.0;
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = new double[] { Math.abs(driveSim.getCurrentDrawAmps()) };
        inputs.driveTempCelsius = new double[] {};

        inputs.angleAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.angleInternalPositionRad = turnRelativePositionRad;
        inputs.angleInternalVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        inputs.angleAppliedVolts = turnAppliedVolts;
        inputs.angleCurrentAmps = new double[] { Math.abs(turnSim.getCurrentDrawAmps()) };
        inputs.angleTempCelsius = new double[] {};
    }

    public void setDriveVoltage(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveSim.setInputVoltage(driveAppliedVolts / 2.0);
    }

    public void setAngleVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    public void setAnglePosition(double radians) {
        setAngleVoltage(anglePID.calculate(turnAbsolutePositionRad, radians));
    }
}