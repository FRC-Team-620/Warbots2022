package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroTurnTable extends CommandBase {
    LazySusanSubsystem lazySusanSubsystem;
    Rotation2d targetRotation;
    public ZeroTurnTable(LazySusanSubsystem lazySusanSubsystem) {
        addRequirements(lazySusanSubsystem);
        // calSwitch = new DigitalInput(Constants.calSwitchID);
        this.lazySusanSubsystem = lazySusanSubsystem;
        
    }

    @Override
    public void initialize() {
        targetRotation = lazySusanSubsystem.getRotation().minus(Rotation2d.fromDegrees(20));
        lazySusanSubsystem.setModSpeed(0.8);
        lazySusanSubsystem.setTurretPositionDegrees(targetRotation);
        lazySusanSubsystem.setIsCal(false);
    }

    @Override
    public boolean isFinished() {
        // System.out.println( lazySusanSubsystem.atTurretPosition()); //lazySusanSubsystem.calSwitch.get() ||
        // System.out.println(lazySusanSubsystem.islimitSwitchPressed());
        if(lazySusanSubsystem.atTurretPosition() || lazySusanSubsystem.islimitSwitchPressed()) {
            if(lazySusanSubsystem.islimitSwitchPressed()) {
                //lazySusanSubsystem.setIsCal(true);
                lazySusanSubsystem.setHomePosition();
            }
            // return true;
        }
        return false;
        // return true;
    }

    @Override
    public void end(boolean interrupted) {
        // lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation());
        
        // lazySusanSubsystem.stop();
        // lazySusanSubsystem.setModSpeed(1);

        // System.out.println("\n\n\n\t\tOHGOEHGOHWROGHROWGOOBRNBOBN\n\n\n");

        // if (!interrupted) {
        //     lazySusanSubsystem.setHomePosition();
        // }
    }

    @Override
    public void execute() {

    }

}
