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
    protected boolean autoOn;
    protected boolean lowPoweredShot = false;
    
    CANSparkMax lazySusanMotor;

    // turntable
    protected double ticksPerTurntableRotation,angleChangePerTick;
    protected double  currTicksGoal = 0, diffConstLS = 0.023, turntableThresh = 35;
    // shooter
    protected double rpm, bangBangTolerance = 0.01, minRPM = 0, maxRPM = 5500,
            currentSpeed = 0, diffConst = 6 * Math.pow(10, -6), acceleration, 
            input, roundTo, currRPM, deltaTheta;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, LazySusanSubsystem lazySusanSubsystem, XboxController operatorXbox, XboxController driverXbox) {
        // timer.start();
        // prevTime = timer.get();
        this.ticksPerTurntableRotation = lazySusanSubsystem.getTicksPerMotorRotation()*Constants.motorRotationsPerTurntableRotation;
        this.angleChangePerTick = 2*Math.PI/this.ticksPerTurntableRotation;        SmartDashboard.putNumber("Round to: ", 5);
        SmartDashboard.putNumber("Set default RPM: ", 0);
        // SmartDashboard.putNumber("Change angle: ", 0);
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.operatorXbox = operatorXbox;
        this.driverXbox = driverXbox;
        this.lazySusanSubsystem = lazySusanSubsystem;
        lazySusanMotor = lazySusanSubsystem.getLazySusanMotor();
        lazySusanEnc = lazySusanSubsystem.getLazySusanEncoder();
        autoOn = false;
        
        
    }

    @Override
    public void execute() {
        boolean hasTarget = entryHasTarget.getDouble(0.0) == 1.0;
        double x = entryX.getDouble(0.0);
        double y = entryY.getDouble(0.0);
        double area = entryArea.getDouble(0.0);

        double borkedJoystickDeadband = 0.05;
        input = operatorXbox.getRightY();
        if (Math.abs(input) < borkedJoystickDeadband) {
            input = 0;
        }
        roundTo = SmartDashboard.getNumber("Round to: ", 5);
        currRPM = shooterSubsystem.getRPM();

        // post to smart dashboard periodically
        SmartDashboard.putBoolean("LimelightHasTarget", hasTarget);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        // new math for static limelight shooting
        double tempDist = getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        double tempRPM = metersToRPM(tempDist * Constants.metersToFeet);
        System.out.println("Dist: " + tempDist);
        
        System.out.println(lazySusanEnc.getPosition());
        if(operatorXbox.getRightBumper() || autoOn) {
            table.getEntry("ledMode").setNumber(3);
            double speeeeed = -x*diffConstLS; // this is speed
            // Making sure it's within the provided threshholds
            if ((lazySusanEnc.getPosition() <= -turntableThresh && speeeeed < 0)
                || (lazySusanEnc.getPosition() >= turntableThresh && speeeeed > 0)) {
                speeeeed = 0;
            }
            lazySusanMotor.set(speeeeed);
        } else if (Math.abs(operatorXbox.getLeftX()) > 0) {
            double speed = -operatorXbox.getLeftX()/2.5;
            if ((lazySusanEnc.getPosition() <= -turntableThresh && speed < 0)
                || (lazySusanEnc.getPosition() >= turntableThresh && speed > 0)) {
                speed = 0;
            }
            lazySusanMotor.set(speed);

        } else {
            table.getEntry("ledMode").setNumber(1);
            lazySusanMotor.set(0);

        }
        if (driverXbox.getBButton()) {
            lowPoweredShot = true;
        } else {
            lowPoweredShot = false;
        }
        if (input > 0) {
            setRPM(input * (maxRPM - minRPM) + minRPM); 
        } else if(hasTarget && operatorXbox.getRightBumper()) {
             setRPM(tempRPM);
        } else {
            if (lowPoweredShot) {
                setRPM(Constants.lowPoweredShotRPM);
            } else {
                setRPM(SmartDashboard.getNumber("Set default RPM: ", 0));
            }
            
            if (getRPM() > maxRPM)
                setRPM(maxRPM);
            else if (getRPM() < minRPM)
                setRPM(minRPM);
        }
        acceleration = diffConst * (getRPM() - currRPM);
        SmartDashboard.putNumber("Acceleration: ", acceleration);
        // if(Math.abs(rpm-currRPM) > (bangBangTolerance * rpm))
        setShooterSpeedAndUpdate(currentSpeed + acceleration);

        SmartDashboard.putNumber("Shooter RPM: ",
                roundUpToNearestMultiple(currRPM, (int) roundTo));
    }

    public double getDistanceInMeters(double a1, double a2, double h1, double h2) {
        return (h2 - h1) / Math.tan((a1 + a2) * (Math.PI/180));
    }

    public double metersToRPM(double meters) {
        double distanceInFeet = 3.28084 * meters;
        return 11.73708 * distanceInFeet + 1632.61502;
    }

    static long roundUpToNearestMultiple(double input, int step) {
        int i = (int) Math.ceil(input);
        return ((i + step - 1) / step) * step;
    }

    public void setShooterSpeedAndUpdate(double speed) {
        shooterSubsystem.setShooterSpeed(speed);
        currentSpeed = speed;
    }

    public void setRPM(double rpm) {
        this.rpm = rpm;
    }
    public double getRPM() {
        return this.rpm;
    }

    public void setAutoOn(boolean x) {
        autoOn = x;
    }
    public NetworkTable getTable() {
        return table;
    }
    

    @Override
    public boolean isFinished() {
        return false;
    }
}