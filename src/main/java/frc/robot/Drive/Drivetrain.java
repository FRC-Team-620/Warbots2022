package frc.robot.Drive;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

    private CANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private DifferentialDrive drive;
    private RelativeEncoder leftFrontEncoder, rightFrontEncoder;
    private DifferentialDriveOdometry odometry;

    // PUBLIC
    public Drivetrain() {
        initMotors();
        initDrive();
        initSensors();
    }

    public void setFudgeSpeed(double fudgeFactor) {
        frontLeftMotor.set(frontLeftMotor.get() + fudgeFactor);
    }
    public void curvatureInput(double speed, double rotation, boolean quickTurn) {
        drive.curvatureDrive(speed, -rotation, quickTurn);
    }

    public void setEncoderPos(double pos) {
        frontLeftMotor.getEncoder().setPosition(pos);
        frontRightMotor.getEncoder().setPosition(pos);
        rearLeftMotor.getEncoder().setPosition(pos);
        rearRightMotor.getEncoder().setPosition(pos);

    }

    public void tankDriveSet(double leftSpeed, double rightSpeed) {
        frontLeftMotor.set(leftSpeed);
        frontRightMotor.set(rightSpeed);
        // drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDriveSet(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    // average the two encoders for this later
    public double getLeftSpeed() { // TODO: Remove Use Odom class
        return leftFrontEncoder.getVelocity();
    }

    public double getLeftPosition() {// TODO: Remove Use Odom class
        return leftFrontEncoder.getPosition();
    }

    public double getRightSpeed() {//TODO: Remove Use Odom class
    return rightFrontEncoder.getVelocity();
    }

    public double getRightPosition() {//TODO: Remove Use Odom class
    return rightFrontEncoder.getPosition();
    }

    public void setBrake(boolean brake) {
        IdleMode mode;
        if (brake) {
            mode = IdleMode.kBrake;
        } else {
            mode = IdleMode.kCoast;
        }

        frontRightMotor.setIdleMode(mode);
        frontLeftMotor.setIdleMode(mode);
        rearRightMotor.setIdleMode(mode);
        rearLeftMotor.setIdleMode(mode);

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // PRIVATE
    private void initMotors() {
        frontLeftMotor = new CANSparkMax(DriveConstants.frontLeftMotorID, MotorType.kBrushless);
        frontRightMotor = new CANSparkMax(DriveConstants.frontRightMotorID, MotorType.kBrushless);
        rearLeftMotor = new CANSparkMax(DriveConstants.rearLeftMotorID, MotorType.kBrushless);
        rearRightMotor = new CANSparkMax(DriveConstants.rearRightMotorID, MotorType.kBrushless);

        rearRightMotor.follow(frontRightMotor);
        rearLeftMotor.follow(frontLeftMotor);

        frontRightMotor.setInverted(false);
        frontLeftMotor.setInverted(true);

        int currentLimit = Constants.NEO_CURRENT_LIMIT;
        frontLeftMotor.setSmartCurrentLimit(currentLimit);
        frontRightMotor.setSmartCurrentLimit(currentLimit);
        rearLeftMotor.setSmartCurrentLimit(currentLimit);
        rearRightMotor.setSmartCurrentLimit(currentLimit);

        frontRightMotor.setOpenLoopRampRate(DriveConstants.driveRampRate);
        frontLeftMotor.setOpenLoopRampRate(DriveConstants.driveRampRate);

        setBrake(true);
    }

    private void initDrive() {
        drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        drive.setMaxOutput(DriveConstants.driveTrainMaxSpeed);
    }

    private void initSensors() {
        leftFrontEncoder = frontLeftMotor.getEncoder();
        rightFrontEncoder = frontRightMotor.getEncoder();
    }

    public double getDrawnCurrentAmps() {
        return this.frontLeftMotor.getOutputCurrent() + this.frontRightMotor.getOutputCurrent()
                + this.rearLeftMotor.getOutputCurrent() + this.rearRightMotor.getOutputCurrent();
    }
}
