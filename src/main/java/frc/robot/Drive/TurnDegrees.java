package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class TurnDegrees extends CommandBase {
    Drivetrain drivetrain;
    double deltaDegrees, targetDegrees, tolerance = 0.035;
    
    public TurnDegrees(Drivetrain drivetrain, double deltaDegrees) {
        this.drivetrain = drivetrain;
        this.deltaDegrees = deltaDegrees;
    }

    public void initialize() {
        this.targetDegrees = this.drivetrain.getHeading() + this.deltaDegrees;
    }

    public void execute() {
        double speed = Constants.diffConstTurn*(this.targetDegrees-this.drivetrain.getHeading());
        drivetrain.tankDriveSet(-speed, speed);
    }

    public boolean isFinished() {
        double angle = this.drivetrain.getHeading();
        return angle > (1-this.tolerance)*this.targetDegrees && angle < (1+this.tolerance)*this.targetDegrees;
    }
}
