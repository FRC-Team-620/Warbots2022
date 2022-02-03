package frc.robot.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterCommand extends CommandBase {
    protected XboxController driverXbox;
    protected ShooterSubsystem shooterSubsystem;

    // protected Timer timer = new Timer(); // For dealing with RPM
    // protected long prevRotations = 0;
    // protected double secondsTimestep = 0.2, prevTime, currentTime, prevRPM = 0;

    protected NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    protected NetworkTableEntry entryHasTarget = table.getEntry("tv");
    protected NetworkTableEntry entryX = table.getEntry("tx");
    protected NetworkTableEntry entryY = table.getEntry("ty");
    protected NetworkTableEntry entryArea = table.getEntry("ta");

    protected double bangBangTolerance = 0.05, intermittantSpeed = 0.3, minRPM = 1000, maxRPM = 3000;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, XboxController driverXbox) {

        // timer.start();
        // prevTime = timer.get();

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
        // // double rpm = driverXbox.getRightY() * (maxRPM - minRPM) + minRPM;
        // // if (shooterSubsystem.getRPM() < rpm - bangBangTolerance * rpm)
        // // shooterSubsystem.setShooterSpeed(intermittantSpeed);
        // // else
        // // shooterSubsystem.setShooterSpeed(0);

        // // System.out.println(shooterSubsystem.getRPM());

        // SmartDashboard.putNumber("Shooter RPM: ", shooterSubsystem.getRPM());

        // currentTime = timer.get(); // milliseconds to seconds

        double speed = driverXbox.getRightTriggerAxis();
        shooterSubsystem.setShooterSpeed(speed);
        // double rpm = driverXbox.getRightY() * (maxRPM - minRPM) + minRPM;
        // if (shooterSubsystem.getRPM() < rpm - bangBangTolerance * rpm)
        // shooterSubsystem.setShooterSpeed(intermittantSpeed);
        // else
        // shooterSubsystem.setShooterSpeed(0);

        // System.out.println(shooterSubsystem.getRPM());

        // if(currentTime - prevTime >= secondsTimestep) {
        // prevTime = currentTime;
        // prevRPM = shooterSubsystem.getRPM(prevRotations, secondsTimestep);
        // prevRotations = shooterSubsystem.getTotalWheelRotations();
        // }
        SmartDashboard.putNumber("Shooter RPM: ", shooterSubsystem.getRPM());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}