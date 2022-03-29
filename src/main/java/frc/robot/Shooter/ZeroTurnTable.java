package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class ZeroTurnTable extends CommandBase {
    LazySusanSubsystem lazySusanSubsystem;
    DigitalInput calSwitch;
    Rotation2d targetRotation;
    public ZeroTurnTable(LazySusanSubsystem lazySusanSubsystem) {
        calSwitch = new DigitalInput(Constants.calSwitchID);
        this.lazySusanSubsystem = lazySusanSubsystem;
    }

    @Override
    public void initialize() {
        targetRotation = lazySusanSubsystem.getRotation().minus(Rotation2d.fromDegrees(10));
        lazySusanSubsystem.setModSpeed(0.2);
        lazySusanSubsystem.setTurretPositionDegrees(targetRotation);

    }

    @Override
    public boolean isFinished() {
        return calSwitch.get() || lazySusanSubsystem.atTurretPosition();
    }

    @Override
    public void end(boolean interrupted) {
        lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation());
        lazySusanSubsystem.setModSpeed(1);
        if (!interrupted) {
            lazySusanSubsystem.setHomePosition();
        }
    }
    


    @Override
    public void execute() {

    }

}
