package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveInDirection extends CommandBase {
    private final Drivetrain drivetrain;
    private final double direction;
    private final double speed;
    private final double targetFrames;
    private double frames;

    public enum Direction {
        FORWARDS(1), 
        BACKWARDS(-1);

        private final double direction;

        private Direction(double direction) {
            this.direction = direction;
        }

        private double get() {
            return this.direction;
        }
    }

    public DriveInDirection(Drivetrain drivetrain, Direction direction, double speed, double targetFrames) {
        this.drivetrain = drivetrain;
        this.direction = direction.get();
        this.speed = speed;
        this.targetFrames = targetFrames;
        addRequirements(this.drivetrain);
    }

    public DriveInDirection(Drivetrain drivetrain, Direction direction) {
        this(drivetrain, direction, 0.5, 60); //40 frames = 7ft
    }
    
    @Override
    public void initialize() {
        this.frames = 0;
    }

    @Override
    public void execute() {
        drivetrain.curvatureInput(this.direction * this.speed, 0, false);
        this.frames++;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.setBrake(true);
    } 

    @Override
    public boolean isFinished() {
        return this.frames > this.targetFrames;
    }
}
