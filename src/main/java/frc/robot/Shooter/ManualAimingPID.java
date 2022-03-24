package frc.robot.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualAimingPID extends CommandBase {
    private LazySusanSubsystem lazySusanSubsystem;
    private XboxController operatorXbox;
    private final double maxSpeed = 3; 
    public ManualAimingPID(LazySusanSubsystem lazySusanSubsystem, XboxController operatorXbox) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.operatorXbox = operatorXbox;
        addRequirements(lazySusanSubsystem);
    }

    @Override
    public void execute() {
        lazySusanSubsystem.setTurretPositionDegrees(lazySusanSubsystem.getSetpointDegrees() + maxSpeed * operatorXbox.getLeftX());
        //lazySusanSubsystem.setTurretPosition(operatorXbox.getLeftX() * lazySusanSubsystem.highLimit);
    }

}
