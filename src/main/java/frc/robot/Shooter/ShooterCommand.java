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

    protected double bangBangTolerance = 0.05, intermittantAcceleration = 0.01, 
        intermittantDeceleration = 0.03, minRPM = 1000, maxRPM = 5000, currentSpeed = 0;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, XboxController driverXbox) {

        // timer.start();
        // prevTime = timer.get();

        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.driverXbox = driverXbox;
    }

    @Override
    public void execute() {
        // double speed = driverXbox.getRightTriggerAxis();
        // shooterSubsystem.setShooterSpeed(speed);
        double rpm = driverXbox.getRightY() * (maxRPM - minRPM) + minRPM;
        rpm = roundUpToNearestMultiple(rpm, 50);
        if (shooterSubsystem.getRPM() < rpm - bangBangTolerance * rpm)
            setShooterSpeedAndUpdate(currentSpeed + intermittantAcceleration);
        else
            setShooterSpeedAndUpdate(currentSpeed - intermittantDeceleration);

        // // System.out.println(shooterSubsystem.getRPM());

        // SmartDashboard.putNumber("Shooter RPM: ", shooterSubsystem.getRPM());

        // currentTime = timer.get(); // milliseconds to seconds

        // System.out.println(shooterSubsystem.getRPM());

        // if(currentTime - prevTime >= secondsTimestep) {
        // prevTime = currentTime;
        // prevRPM = shooterSubsystem.getRPM(prevRotations, secondsTimestep);
        // prevRotations = shooterSubsystem.getTotalWheelRotations();
        // }
        SmartDashboard.putNumber("Shooter RPM: ", shooterSubsystem.getRPM());
    }

    static long roundUpToNearestMultiple(double input, long step) {
        long i = (long) Math.ceil(input);
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