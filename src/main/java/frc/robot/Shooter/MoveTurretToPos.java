package frc.robot.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class MoveTurretToPos extends CommandBase {
    LazySusanSubsystem lazySusanSubsystem;
    private double x;
    public MoveTurretToPos(LazySusanSubsystem lazySusanSubsystem) {
        addRequirements(lazySusanSubsystem);
        this.lazySusanSubsystem = lazySusanSubsystem;
        x = Constants.stowedDegrees;
    }

    @Override
    public void initialize(){
        SmartDashboard.putNumber("TestPos: ", x);
    }

    @Override
    public void execute() {
        x = SmartDashboard.getNumber("TestPos: ", Constants.stowedDegrees);
        lazySusanSubsystem.setTurretPositionDegrees(Rotation2d.fromDegrees(x));
        
    }

    
    
}
