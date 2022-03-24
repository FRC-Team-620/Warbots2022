package frc.robot.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualAimingPID extends CommandBase {
    private LazySusanSubsystem lazySusanSubsystem;
    private XboxController operatorXbox;
    public ManualAimingPID(LazySusanSubsystem lazySusanSubsystem, XboxController operatorXbox) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.operatorXbox = operatorXbox;
    }

    @Override
    public void execute() {
        lazySusanSubsystem.setTurretPosition(operatorXbox.getLeftX() * lazySusanSubsystem.highLimit);
    }

}
