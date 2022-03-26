package frc.robot.Shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestSetpointSpinUp extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    public TestSetpointSpinUp(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void execute() {
        shooterSubsystem.setTargetRPM(shooterSubsystem.getTestRPM());
        System.out.println(shooterSubsystem.getTestRPM());
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setTargetRPM(0);
    }


}
