package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight;

public class ShooterCommand extends CommandBase {
    protected XboxController operatorXbox;
    protected XboxController driverXbox;
    protected ShooterSubsystem shooterSubsystem;
    protected LazySusanSubsystem lazySusanSubsystem;
    // protected double bangBangTolerance = 0.05, intermittantSpeed = 0.3, minRPM =
    // 1000, maxRPM = 3000;

    // protected Timer timer = new Timer(); // For dealing with RPM
    // protected long prevRotations = 0;
    // protected double secondsTimestep = 0.2, prevTime, currentTime, prevRPM = 0;

    protected NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    protected NetworkTableEntry entryHasTarget = table.getEntry("tv");
    protected NetworkTableEntry entryX = table.getEntry("tx");
    protected NetworkTableEntry entryY = table.getEntry("ty");
    protected NetworkTableEntry entryArea = table.getEntry("ta");
    protected double turretControlConstant= 0.01;
    protected RelativeEncoder lazySusanEnc;
    
    CANSparkMax lazySusanMotor;

    // turntable
    protected double ticksPerTurntableRotation, angleChangePerTick;
    protected double  currTicksGoal = 0;
    protected double inputOpRight = 0;
    // shooter
    protected double rpm, acceleration, input, roundTo, currRPM, deltaTheta;

    protected LimeLight limelight;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, LazySusanSubsystem lazySusanSubsystem, XboxController operatorXbox, XboxController driverXbox) {
        // timer.start();
        // prevTime = timer.get();
        this.ticksPerTurntableRotation = lazySusanSubsystem.getTicksPerMotorRotation()*Constants.motorRotationsPerTurntableRotation;
        this.angleChangePerTick = 2 * Math.PI / this.ticksPerTurntableRotation;
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
        boolean hasTarget = entryHasTarget.getDouble(0) == 1;
        double x = entryX.getDouble(0);
        double y = entryY.getDouble(0);

        double borkedJoystickDeadband = 0.05;
        inputOpRight = operatorXbox.getLeftX();

        if (Math.abs(inputOpRight) < borkedJoystickDeadband) {
            inputOpRight = 0;
        }

        roundTo = SmartDashboard.getNumber("Round to: ", 5);
        currRPM = shooterSubsystem.getRPM();

        // post to smart dashboard periodically
        SmartDashboard.putBoolean("LimelightHasTarget", hasTarget);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);

        // math for static limelight shooting
        double tempDist = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, y, 
        Constants.limelightHeight, Constants.hubHeight);
        double tempRPM = ShooterMath.metersToRPM(tempDist);
        System.out.println("Dist: " + tempDist);
        System.out.println("RPM: " + tempRPM);

        if(Math.abs(operatorXbox.getLeftTriggerAxis()) > 0) {
            table.getEntry("ledMode").setNumber(3); // Force LimeLight on
            double speed = -(x-Constants.leftBias)*Constants.diffConstLS; // this is speed
            // Making sure it's within the provided threshholds
            if (!ShooterMath.inThreshold(speed > 0, lazySusanEnc.getPosition(), Constants.turntableThresh)) {
                speed = 0;
            }
            lazySusanMotor.set(speed);
        } else if (Math.abs(inputOpRight) > 0) {
            table.getEntry("ledMode").setNumber(1); // Force LimeLight off
            double speed = -inputOpRight/1.4;
            System.out.println("JIWJF_" + lazySusanEnc.getPosition());
            if (!ShooterMath.inThreshold(speed > 0, lazySusanEnc.getPosition(), Constants.turntableThresh)) {
                speed = 0;
            }
            lazySusanMotor.set(speed);
        } else {
            table.getEntry("ledMode").setNumber(1);
            lazySusanMotor.set(0);
        }
        
        if(hasTarget && Math.abs(operatorXbox.getLeftTriggerAxis()) > 0) {
             setRPM(tempRPM);
        } else {
            if (driverXbox.getYButton()) { // The Y Button triggers the low powered shot
                setRPM(Constants.lowPoweredShotRPM);
            } else {
                setRPM(SmartDashboard.getNumber("Set default RPM: ", 0));
            }
            
            if (getRPM() > Constants.maxShooterRPM)
                setRPM(Constants.maxShooterRPM);
            else if (getRPM() < Constants.minShooterRPM)
                setRPM(Constants.minShooterRPM);
        }
        acceleration = Constants.diffConstShooter * (getRPM() - currRPM);
        SmartDashboard.putNumber("Acceleration: ", acceleration);
        shooterSubsystem.setSpeed(shooterSubsystem.getSpeed() + acceleration);
        
        if (rpm == 0) {
            shooterSubsystem.setSpeed(0);
        }

        SmartDashboard.putNumber("Shooter RPM: ",
                ShooterMath.roundUpToNearestMultiple(currRPM, (int) roundTo));
        
        if (operatorXbox.getAButtonPressed()) {
            lazySusanSubsystem.getLazySusanEncoder().setPosition(0);
        }
    }

    public void setRPM(double rpm) {
        this.rpm = rpm;
    }

    public double getRPM() {
        return this.rpm;
    }

    public NetworkTable getTable() {
        return table;
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}