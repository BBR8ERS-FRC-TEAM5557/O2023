package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ElevatorIO {
    /** Contains all of the input data received from hardware. */
    public static class ElevatorIOInputs implements LoggableInputs {
        public double ElevatorHeightInches = 0.0;
        public double ElevatorVelocityInchesPerSecond = 0.0;
        public boolean ElevatorAtLowerLimit = false;
        public double ElevatorAppliedVolts = 0.0;
        public double[] ElevatorCurrentAmps = new double[] {0.0};
        public double[] ElevatorTempCelsius = new double[] {0.0};

        public void toLog(LogTable table) {
            table.put("ElevatorHeightInches", ElevatorHeightInches);
            table.put("ElevatorVelocityInchesPerSecond", ElevatorVelocityInchesPerSecond);
            table.put("ElevatorAppliedVolts", ElevatorAppliedVolts);
            table.put("ElevatorCurrentAmps", ElevatorCurrentAmps);
            table.put("ElevatorTempCelsius", ElevatorTempCelsius);
        }

        public void fromLog(LogTable table) {
            ElevatorHeightInches = table.getDouble("ElevatorHeightInches", ElevatorHeightInches);
            ElevatorVelocityInchesPerSecond = table.getDouble("ElevatorVelocityRPM", ElevatorVelocityInchesPerSecond);
            ElevatorAppliedVolts = table.getDouble("ElevatorAppliedVolts", ElevatorAppliedVolts);
            ElevatorCurrentAmps = table.getDoubleArray("ElevatorCurrentAmps", ElevatorCurrentAmps);
            ElevatorTempCelsius = table.getDoubleArray("ElevatorTempCelsius", ElevatorTempCelsius);
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ElevatorIOInputs inputs) {
    }

    /** Run the Elevator open loop at the specified voltage. */
    public default void setVoltage(double volts) {
    }

    public default void setPercent(double percent) {
    }

    public default void setHeightInches(double targetHeightInches, double targetVelocityInchesPerSec) {
    }

    public default void resetSensorPosition(double heightInches) {
    }

    public default void brakeOff() {
    }

    public default void brakeOn() {
    }

    public default void shouldEnableUpperLimit(boolean value) {
    }

    public default void shouldEnableLowerLimit(boolean value) {
    }
}
