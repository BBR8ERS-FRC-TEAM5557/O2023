package frc.robot;

public class Constants {

    public static boolean kIsReal = Robot.isReal();
    public static boolean kTuningMode = true;

    public class RobotMap {
        public static final int kFLDriveMotor = 21;
        public static final int kFLTurnMotor = 22;
        public static final int kFLCancoder = 12;
        public static final double kFLOffset = -4.58;
        
        public static final int kFRDriveMotor = 27;
        public static final int kFRTurnMotor = 28;
        public static final int kFRCancoder = 13;
        public static final double kFROffset = -344.74;

        public static final int kBLDriveMotor = 23;
        public static final int kBLTurnMotor = 24;
        public static final int kBLCancoder = 11;
        public static final double kBLOffset = -105.42;
        
        public static final int kBRDriveMotor = 25;
        public static final int kBRTurnMotor = 26;
        public static final int kBRCancoder = 10;
        public static final double kBROffset = -81.18;
        
        public static final int kElevatorMasterMotor = 31;
        public static final int kElevatorSlaveMotor = 0;

        public static final int kRollerMotor = 0;

        public static final int kWristMotor = 40;


        
        public static final int kLedsDIO = 0;
    }
}
