package frc.robot.Drive;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveForwardsEncoder extends CommandBase {
    private Drivetrain drivetrain;
    private double frames;
    private double encoders;
    public DriveForwardsEncoder(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        this.frames = 0;
    }

    @Override
    public void execute() {
        drivetrain.curvatureInput(0.5, 0, false);
        frames++;
    }
    @Override
    public void end(boolean interrupted) {
        this.drivetrain.setMotorMode(IdleMode.kBrake);
    } 

    @Override
    public boolean isFinished() {//40 = 7ft
        // return frames > 60;
        return drivetrain.getEncoderPos(2) > 20;
    }
}

