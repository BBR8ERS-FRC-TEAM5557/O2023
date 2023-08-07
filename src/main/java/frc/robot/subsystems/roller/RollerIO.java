package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface RollerIO {

    public static class RollerIOInputs implements LoggableInputs {
        
        public double rollerVelocityRPM = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double[] rollerCurrentAmps = new double[] {};
        public double[] rollerTempCelcius = new double [] {};


        @Override
        public void toLog(LogTable table) {
            table.put("RollerVelocityRPM", rollerVelocityRPM);
            table.put("RollerAppliedVolts", rollerAppliedVolts);
            table.put("RollerCurrentAmps", rollerCurrentAmps);
            table.put("rollerTempCelcius", rollerTempCelcius);
        }

        @Override
        public void fromLog(LogTable table) {
            rollerVelocityRPM = table.getDouble("RollerVelocityRPM", rollerVelocityRPM);
            rollerAppliedVolts = table.getDouble("RollerAppliedVolts", rollerAppliedVolts);
            rollerCurrentAmps = table.getDoubleArray("RollerCurrentAmps", rollerCurrentAmps);
            rollerTempCelcius = table.getDoubleArray("rollerTempCelcius", rollerTempCelcius);
        }
    }

    public default void updateInputs(RollerIOInputs inputs) {}

    public default void setRollerVoltage(double volts) {}

}
