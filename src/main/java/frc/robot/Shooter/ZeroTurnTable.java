package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroTurnTable extends CommandBase {
    LazySusanSubsystem lazySusanSubsystem;
    double targetRotation;
    Timer timer;
    public ZeroTurnTable(LazySusanSubsystem lazySusanSubsystem) {
        addRequirements(lazySusanSubsystem);
        // calSwitch = new DigitalInput(Constants.calSwitchID);
        this.lazySusanSubsystem = lazySusanSubsystem;
        
    }

    @Override
    public void initialize() {
        lazySusanSubsystem.setIsGyroLocking(false);
        lazySusanSubsystem.setIsHubTracking(false);
        lazySusanSubsystem.setSmartCurrentLimit(5);
        //lazySusanSubsystem.setMotorSpeed(0.1);
        targetRotation = lazySusanSubsystem.getRotationDegrees() - 189.5;
        lazySusanSubsystem.setModSpeed(0.17);//0.4
        lazySusanSubsystem.setTurretPositionDegrees(targetRotation);
        lazySusanSubsystem.setIsCal(false);
        timer = new Timer();
    }

    @Override
    public boolean isFinished() {
        // System.out.println( lazySusanSubsystem.atTurretPosition()); //lazySusanSubsystem.calSwitch.get() ||
        // System.out.println(lazySusanSubsystem.islimitSwitchPressed());
        if (lazySusanSubsystem.atTurretPosition()) {
            timer.start();
        }
        if (timer.hasElapsed(1)) {
            return true;
        }

        if(lazySusanSubsystem.islimitSwitchPressed()) {
            lazySusanSubsystem.setHomePosition();
            // if(lazySusanSubsystem.getRotation().getDegrees() < 100) {
            //     //lazySusanSubsystem.setIsCal(true);
            //     lazySusanSubsystem.setHomePosition();
            // }
            return true;
        }
        return false;
        // return true;
    }

    @Override
    public void end(boolean interrupted) {
        // lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation());
        
        // lazySusanSubsystem.stop();
        lazySusanSubsystem.setModSpeed(1);
        lazySusanSubsystem.setSmartCurrentLimit(18);
        lazySusanSubsystem.setIsGyroLocking(true);
        lazySusanSubsystem.setIsHubTracking(true);
        // lazySusanSubsystem.setMotorSpeed(0);
        // lazySusanSubsystem.setHomePosition();
        

        // System.out.println("\n\n\n\t\tOHGOEHGOHWROGHROWGOOBRNBOBN\n\n\n");

        // if (!interrupted) {
        //     lazySusanSubsystem.setHomePosition();
        // }
    }
}
