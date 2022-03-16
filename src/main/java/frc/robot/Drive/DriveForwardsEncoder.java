package frc.robot.Drive;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardsEncoder extends CommandBase {
    private Drivetrain drivetrain;
    private double targetDistance = 2.7; // this number is somewhat close to meters (within a few centimeters)
    private double error;
    private boolean finished = false;

    public DriveForwardsEncoder(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double encoderVal = drivetrain.getEncoderPos(2);
        error = targetDistance - encoderVal;

        if (!withinError()) {
            double kP = 0.8; // this is not tuned by any means, just enough to get it working
            double input = error * kP;
            drivetrain.curvatureInput(input, 0, false);
            // SmartDashboard.putNumber("Encoder", drivetrain.getEncoderPos(2));
            // SmartDashboard.putNumber("Input", input);
        } else {
            // ik this is dumb, just leave it for now. it works.
            Timer.delay(1.5);
            finished = true;
        }

    }

    private boolean withinError() {
        return error < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.setMotorMode(IdleMode.kBrake);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
