package frc.robot.Drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private CANSparkMax frontLeftMotor, frontRightMotor, rearLeftMotor, rearRightMotor;
    private DifferentialDrive drive;
    private AHRS navx;
    private RelativeEncoder leftFrontEncoder, rightFrontEncoder;

    // PUBLIC
    public Drivetrain() {
        initMotors();
        initDrive();
        initSensors();
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


    public double getHeading() {
        return navx.getAngle();
    }

    public void tankDriveSet(double leftSpeed, double rightSpeed) {
        frontLeftMotor.set(leftSpeed);
        frontRightMotor.set(rightSpeed);
        //drive.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDriveSet(double speed, double rotation) {
        drive.arcadeDrive(speed, rotation);
    }
    
    // average the two encoders for this later
    public double getLeftSpeed() {
        return leftFrontEncoder.getVelocity();
    }

    public double getLeftPosition() {
        return leftFrontEncoder.getPosition();
    }

    public double getRightSpeed() {
        return rightFrontEncoder.getVelocity();
    }

    public double getRightPosition() {
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

        frontRightMotor.setOpenLoopRampRate(DriveConstants.driveRampRate);
        frontLeftMotor.setOpenLoopRampRate(DriveConstants.driveRampRate);

        setBrake(true);
    }

    private void initDrive() {
        drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
        drive.setMaxOutput(DriveConstants.driveTrainMaxSpeed);
    }

    private void initSensors() {
        navx = new AHRS(Port.kMXP);

        leftFrontEncoder = frontLeftMotor.getEncoder();
        rightFrontEncoder = frontRightMotor.getEncoder();
    }

}
