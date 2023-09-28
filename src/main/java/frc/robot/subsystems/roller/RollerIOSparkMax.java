package frc.robot.subsystems.roller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;

import static frc.robot.subsystems.roller.RollerConstants.*;

public class RollerIOSparkMax implements RollerIO {

    private CANSparkMax motor;
    
    public RollerIOSparkMax() {
        motor = SparkMaxFactory.createNEO(kRollerMotorConfiguration);
        BurnManager.burnFlash(motor);
    }

    public void updateInputs(RollerIOInputs inputs) {
        inputs.rollerAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.rollerCurrentAmps = new double[] {motor.getOutputCurrent()};
        inputs.rollerVelocityRPM = motor.getEncoder().getVelocity();
        inputs.rollerTempCelcius = new double[] {motor.getMotorTemperature()};
    }

    public void setRollerVoltage(double value) {
        motor.getPIDController().setReference(value, ControlType.kVoltage);
    }
}
