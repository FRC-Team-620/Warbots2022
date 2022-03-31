package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroTurnTable extends CommandBase {
    LazySusanSubsystem lazySusanSubsystem;
    Rotation2d targetRotation;
    public ZeroTurnTable(LazySusanSubsystem lazySusanSubsystem) {
        // calSwitch = new DigitalInput(Constants.calSwitchID);
        this.lazySusanSubsystem = lazySusanSubsystem;
    }

    @Override
    public void initialize() {
        targetRotation = lazySusanSubsystem.getRotation().plus(Rotation2d.fromDegrees(10));
        lazySusanSubsystem.setModSpeed(0.8);
        lazySusanSubsystem.setTurretPositionDegrees(targetRotation);

    }

    @Override
    public boolean isFinished() {
        System.out.println( lazySusanSubsystem.atTurretPosition()); //lazySusanSubsystem.calSwitch.get() ||
        return  lazySusanSubsystem.atTurretPosition() || lazySusanSubsystem.islimitSwitchPressed();
        // return true;
    }

    @Override
    public void end(boolean interrupted) {
        // lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getRotation());
        lazySusanSubsystem.stop();
        lazySusanSubsystem.setModSpeed(1);
        if (!interrupted) {
            lazySusanSubsystem.setHomePosition();
        }
    }

    @Override
    public void execute() {

    }

}
