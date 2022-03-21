package frc.robot.Drive;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drive.Drivetrain.Motor;

public class DriveForwardsEncoder extends CommandBase {
    private Drivetrain drivetrain;
    private double targetDistance; // this number is somewhat close to meters (within a few centimeters)
    private double error;
    private boolean finished = false;

    public DriveForwardsEncoder(Drivetrain drivetrain, double targetDistance) {
        this.drivetrain = drivetrain;
        this.targetDistance = targetDistance;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        System.out.println("Drive Forwards Auto Running");
        double encoderVal = drivetrain.getEncoderPos(Motor.RIGHT_FRONT);
        error = targetDistance - encoderVal;

        if (!withinError()) {
            double kP = 0.5; // this is not tuned by any means, just enough to get it working
            double input = error * kP;
            drivetrain.curvatureDrive(MathUtil.clamp(input, 0, 0.5), 0, false);
            // SmartDashboard.putNumber("Encoder", drivetrain.getEncoderPos(2));
            // SmartDashboard.putNumber("Input", input);
        } else {
            System.out.println("Exiting DriverForwardsEncoder...");
            // ik this is dumb, just leave it for now. it works.
            Timer.delay(1.5);
            finished = true;
        }

    }

    private boolean withinError() {
        return error < 0.2;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.setAllIdleModes(IdleMode.kBrake);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
