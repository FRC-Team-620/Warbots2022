package frc.robot.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterCommand extends CommandBase {
    protected XboxController driverXbox;
    protected ShooterSubsystem shooterSubsystem;

    // protected Timer timer = new Timer(); // For dealing with RPM
    // protected long prevRotations = 0;
    // protected double secondsTimestep = 0.2, prevTime, currentTime, prevRPM = 0;

    protected double bangBangTolerance = 0.01, minRPM = 0, maxRPM = 5500, 
        currentSpeed = 0, diffConst = 0.000006, acceleration;

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
        // double speed = driverXbox.getRightTriggerAxis();
        // shooterSubsystem.setShooterSpeed(speed);
        double roundTo = SmartDashboard.getNumber("Round to: ", 5);
        double input = driverXbox.getRightTriggerAxis(), rpm;
        if(input > 0)
            rpm = input * (maxRPM - minRPM) + minRPM;
        else {
            rpm = SmartDashboard.getNumber("Set RPM: ", 0);
            if(rpm > maxRPM)
                rpm = maxRPM;
            else if(rpm < minRPM)
                rpm = minRPM;
        }
        double currRPM = shooterSubsystem.getRPM();
        acceleration = diffConst * (rpm - currRPM);
        SmartDashboard.putNumber("Input RPM: ", rpm);
        rpm = roundUpToNearestMultiple(rpm, 50);
        SmartDashboard.putNumber("Acceleration: ", acceleration);

        // if(Math.abs(rpm-currRPM) > (bangBangTolerance * rpm))
        setShooterSpeedAndUpdate(currentSpeed + acceleration);

        SmartDashboard.putNumber("Shooter RPM: ", 
            roundUpToNearestMultiple(shooterSubsystem.getRPM(), (int)roundTo));
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