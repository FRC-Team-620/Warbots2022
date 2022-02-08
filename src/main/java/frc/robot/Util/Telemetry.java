package frc.robot.Util;

public class Telemetry {
    public static boolean drivetrainBool;
    public static boolean xbox;
    public static boolean shooter;
    public static boolean driveWithJoystick;
    public static boolean shooterCommand;

    public static void setdriveTrainBool(boolean driveTrain) {
        drivetrainBool = driveTrain;
    }

    public static void setXbox(boolean xboxA) {
        xbox = xboxA;
    }

    public static void setShooter(boolean shooterA) {
        shooter = shooterA;
    }

    public static void setDriveJoystick(boolean dwj) {
        driveWithJoystick = dwj;
    }

    public static void setShooterCmd(boolean shooterCmd) {
        shooterCommand = shooterCmd;
    }

}
