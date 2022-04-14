package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.RobotMath;

public class ManualAimingPID extends CommandBase {
    private LazySusanSubsystem lazySusanSubsystem;
    private XboxController operatorXbox;
    private final double maxSpeed = 4; 
    public ManualAimingPID(LazySusanSubsystem lazySusanSubsystem, XboxController operatorXbox) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.operatorXbox = operatorXbox;
        addRequirements(lazySusanSubsystem);
    }

    @Override
    public void execute() {
        double controlInput = RobotMath.deadZone(operatorXbox.getLeftX(), 0.1, 0);
        lazySusanSubsystem.setTurretPositionDegrees((lazySusanSubsystem.getDesiredDegrees().minus(Rotation2d.fromDegrees(maxSpeed *controlInput ))));
        //lazySusanSubsystem.setTurretPosition(operatorXbox.getLeftX() * lazySusanSubsystem.highLimit);
    }

}
