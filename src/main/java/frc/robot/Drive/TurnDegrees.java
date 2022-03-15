package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class TurnDegrees extends CommandBase {
    Drivetrain drivetrain;
    double deltaDegrees, targetDegrees, tolerance = 3, maxSpeed = 0.8;
    
    public TurnDegrees(Drivetrain drivetrain, double deltaDegrees) {
        this.drivetrain = drivetrain;
        this.deltaDegrees = deltaDegrees;
    }

    public void initialize() {
        this.targetDegrees = this.drivetrain.getHeading() + this.deltaDegrees;
    }

    public void execute() {
        double speed = Constants.diffConstTurn*(this.targetDegrees-this.drivetrain.getHeading());
        System.out.println("SPEED:" + speed);
        System.out.println("ANGLE: " + this.drivetrain.getHeading());
        System.out.println("TARGET: " + this.targetDegrees);
        int sign = speed < 0 ? -1 : 1;
        speed = sign*Math.min(Math.abs(speed), this.maxSpeed);

        drivetrain.tankDriveSet(-speed, speed);
        // drivetrain.diffDrive.arcadeDrive(0, speed);
    }

    public boolean isFinished() {
        double angle = this.drivetrain.getHeading();
        return angle > this.targetDegrees-this.tolerance && 
            angle < this.targetDegrees+this.tolerance;
    }
}
