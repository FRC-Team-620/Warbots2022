package frc.robot.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterCommand extends CommandBase {
    protected XboxController driverXbox;
    protected ShooterSubsystem shooterSubsystem;
    // protected double bangBangTolerance = 0.05, intermittantSpeed = 0.3, minRPM =
    // 1000, maxRPM = 3000;

    // protected Timer timer = new Timer(); // For dealing with RPM
    // protected long prevRotations = 0;
    // protected double secondsTimestep = 0.2, prevTime, currentTime, prevRPM = 0;

    protected NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    protected NetworkTableEntry entryHasTarget = table.getEntry("tv");
    protected NetworkTableEntry entryX = table.getEntry("tx");
    protected NetworkTableEntry entryY = table.getEntry("ty");
    protected NetworkTableEntry entryArea = table.getEntry("ta");

    protected double bangBangTolerance = 0.01, minRPM = 0, maxRPM = 5500,
            currentSpeed = 0, diffConst = 6 * Math.pow(10, -6), acceleration, 
            rpm, input, roundTo, currRPM;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, XboxController driverXbox) {
        // timer.start();
        // prevTime = timer.get();

        SmartDashboard.putNumber("Round to: ", 5);
        SmartDashboard.putNumber("Set RPM: ", 0);
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.driverXbox = driverXbox;
    }

    @Override
    public void execute() {
        boolean hasTarget = entryHasTarget.getDouble(0.0) == 1.0;
        double x = entryX.getDouble(0.0);
        double y = entryY.getDouble(0.0);
        double area = entryArea.getDouble(0.0);

        input = driverXbox.getRightTriggerAxis();
        roundTo = SmartDashboard.getNumber("Round to: ", 5);
        currRPM = shooterSubsystem.getRPM();

        // post to smart dashboard periodically
        SmartDashboard.putBoolean("LimelightHasTarget", hasTarget);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        // testing is needed to determine the area percentage at the correct distance
        if (hasTarget && area > 0.05) {
            // call motor to rotate shooter to optimal position
        }
        // double speed = driverXbox.getRightTriggerAxis();
        // shooterSubsystem.setShooterSpeed(speed);
        if (input > 0)
            rpm = input * (maxRPM - minRPM) + minRPM;
        else {
            rpm = SmartDashboard.getNumber("Set RPM: ", 0);
            if (rpm > maxRPM)
                rpm = maxRPM;
            else if (rpm < minRPM)
                rpm = minRPM;
        }
        acceleration = diffConst * (rpm - currRPM);
        SmartDashboard.putNumber("Acceleration: ", acceleration);
        // if(Math.abs(rpm-currRPM) > (bangBangTolerance * rpm))
        setShooterSpeedAndUpdate(currentSpeed + acceleration);

        SmartDashboard.putNumber("Shooter RPM: ",
                roundUpToNearestMultiple(currRPM, (int) roundTo));
    }

    static long roundUpToNearestMultiple(double input, int step) {
        int i = (int) Math.ceil(input);
        return ((i + step - 1) / step) * step;
    }

    public void setShooterSpeedAndUpdate(double speed) {
        shooterSubsystem.setShooterSpeed(speed);
        currentSpeed = speed;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}