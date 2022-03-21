package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight;
import frc.robot.Util.LimelightV2;

public class ShooterCommand extends CommandBase {
    protected XboxController operatorXbox;
    protected XboxController driverXbox;
    protected ShooterSubsystem shooterSubsystem;
    protected LazySusanSubsystem lazySusanSubsystem;
    // protected double bangBangTolerance = 0.05, intermittantSpeed = 0.3, minRPM =
    // 1000, maxRPM = 3000;

    // protected Timer timer = new Timer(); // For dealing with RPM
    // protected long prevRotations = 0;
    protected double turretControlConstant= 0.01;
    protected RelativeEncoder lazySusanEnc;
    protected boolean lowPoweredShot = false;
    
    CANSparkMax lazySusanMotor;

    // turntable
    protected double ticksPerTurntableRotation, angleChangePerTick;
    protected double  currTicksGoal = 0;
    protected double inputOpRight = 0;
    // shooter
    protected double targetRpm, currentSpeed = shooterSubsystem.currentSpeed, acceleration, input, roundTo, currRPM, deltaTheta;

    protected LimeLight limelight;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, LazySusanSubsystem lazySusanSubsystem, XboxController operatorXbox, XboxController driverXbox) {
        // timer.start();
        // prevTime = timer.get();
        this.ticksPerTurntableRotation = lazySusanSubsystem.getTicksPerMotorRotation()*Constants.motorRotationsPerTurntableRotation;
        this.angleChangePerTick = 2*Math.PI/this.ticksPerTurntableRotation;
        SmartDashboard.putNumber("Round to: ", 5);
        SmartDashboard.putNumber("Set default RPM: ", 0);
        // SmartDashboard.putNumber("Change angle: ", 0);
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.operatorXbox = operatorXbox;
        this.driverXbox = driverXbox;
        this.lazySusanSubsystem = lazySusanSubsystem;
        lazySusanMotor = lazySusanSubsystem.getLazySusanMotor();
        lazySusanEnc = lazySusanSubsystem.getLazySusanEncoder();
        this.limelight = shooterSubsystem.limeLight;
        //autoOn = false;
    }

    @Override
    public void execute() {
        boolean hasTarget = LimelightV2.targetFound();
        double x = LimelightV2.tX();
        double y = LimelightV2.tY();
        // boolean hasTarget = limelight.hasTarget();
        // double x = limelight.getOffsetX();
        // double y = limelight.getOffsetY();
        // double area = limelight.getArea();

        double borkedJoystickDeadband = 0.05;
        // input = operatorXbox.getRightY();
        inputOpRight = operatorXbox.getLeftX();
        // if (Math.abs(input) < borkedJoystickDeadband) {
        //     input = 0;
        // }
        if (Math.abs(inputOpRight) < borkedJoystickDeadband) {
            inputOpRight = 0;
        }
        roundTo = SmartDashboard.getNumber("Round to: ", 5);
        currRPM = shooterSubsystem.getRPM();

        // post to smart dashboard periodically
        SmartDashboard.putBoolean("LimelightHasTarget", hasTarget);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        //SmartDashboard.putNumber("LimelightArea", area);

        // new math for static limelight shooting
        double tempDist = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        double tempRPM = ShooterMath.metersToRPM(tempDist);
        // System.out.println("Dist: " + tempDist);
        // System.out.println("RPM: " + tempRPM);
        // System.out.println("tempDist: " + tempDist);
        // System.out.println("tempDist * metersToFeet: " + tempDist * Constants.metersToFeet);
        // System.out.println("tempRPM: " + tempRPM);
        // || autoOn
        // System.out.println(lazySusanEnc.getPosition());
        if(Math.abs(operatorXbox.getLeftTriggerAxis()) > 0) { // limelight aim
            LimelightV2.ledOn();
            //table.getEntry("ledMode").setNumber(3);
            // limelight.setLEDMode(LedMode.ON);
            double speed =  -(x-Constants.leftBias)*Constants.diffConstLS; // this is speed
            // Making sure it's within the provided threshholds
            // if ((lazySusanEnc.getPosition() <= -Constants.turntableThresh && speeeeed < 0)
            //    || (lazySusanEnc.getPosition() >= Constants.turntableThresh && speeeeed > 0)) {
            //    speed = 0;
            // }
            // if (!ShooterMath.inThreshold(speed > 0, lazySusanEnc.getPosition())) {
            //     speed = 0;
            // }
            lazySusanMotor.set(speed);
            // System.out.println("Negative X: " + -x);
            // System.out.println("Speed With Constant: " + -x*Constants.diffConstLS);
        } else if (Math.abs(inputOpRight) > 0) {
            LimelightV2.ledOff();
            //table.getEntry("ledMode").setNumber(1);
            // limelight.setLEDMode(LedMode.OFF);
            double speed = -inputOpRight/1.4;
            System.out.println("JIWJF_" + lazySusanEnc.getPosition());
            if ((lazySusanEnc.getPosition() <= -Constants.turntableThresh && speed < 0)
                || (lazySusanEnc.getPosition() >= Constants.turntableThresh && speed > 0)) {
                speed = 0;
            }
            lazySusanMotor.set(speed);
            // System.out.println("Speed Man: "+ speed);
            // System.out.println("Curr Enc Pos: " + lazySusanEnc.getPosition());
            

        } else {
            LimelightV2.ledOff();
            //table.getEntry("ledMode").setNumber(1);
            // limelight.setLEDMode(LedMode.OFF);
            lazySusanMotor.set(0);

        }
        if (driverXbox.getYButton()) {
            System.out.println("UGIOEHGIAENNAEION00");
            lowPoweredShot = true;
        } else {
            lowPoweredShot = false;
        }
        
        if(hasTarget && Math.abs(operatorXbox.getLeftTriggerAxis()) > 0) {
             shooterSubsystem.setTargetRPM(tempRPM);
             //System.out.println("Target + RightBumper: Triggered");
        } else {
            if (lowPoweredShot) {
                shooterSubsystem.setTargetRPM(Constants.lowPoweredShotRPM);
            } else {
                shooterSubsystem.setTargetRPM(SmartDashboard.getNumber("Set default RPM: ", 0));
            }
            
            if (shooterSubsystem.getTargetRPM() > Constants.maxShooterRPM)
            shooterSubsystem.setTargetRPM(Constants.maxShooterRPM);
            else if (shooterSubsystem.getTargetRPM() < Constants.minShooterRPM)
            shooterSubsystem.setTargetRPM(Constants.minShooterRPM);
        }
        // acceleration = Constants.diffConstShooter * (shooterSubsystem.getTargetRPM() - currRPM);
        // SmartDashboard.putNumber("Acceleration: ", acceleration);
        // // if(Math.abs(rpm-currRPM) > (bangBangTolerance * rpm))
        // shooterSubsystem.setShooterSpeedAndUpdate(currentSpeed + acceleration);
        // // System.out.println(rpm);
        // if(targetRpm == 0.0) {
        //     shooterSubsystem.setShooterSpeedAndUpdate(0);
        // }
        SmartDashboard.putNumber("Shooter RPM: ",
                ShooterMath.roundUpToNearestMultiple(currRPM, (int) roundTo));
        
        if (operatorXbox.getAButtonPressed()) {
            lazySusanSubsystem.getLazySusanEncoder().setPosition(0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}