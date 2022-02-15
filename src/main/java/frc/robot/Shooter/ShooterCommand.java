package frc.robot.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterCommand extends CommandBase {
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

    // turntable
    protected double ticksPerTurntableRotation,angleChangePerTick;
    protected double  currTicksGoal = 0, diffConstLS = 0.0002;
    // shooter
    protected double rpm, bangBangTolerance = 0.01, minRPM = 0, maxRPM = 5500,
            currentSpeed = 0, diffConst = 6 * Math.pow(10, -6), acceleration, 
            input, roundTo, currRPM, deltaTheta;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, LazySusanSubsystem lazySusanSubsystem, XboxController driverXbox) {
        // timer.start();
        // prevTime = timer.get();
        this.ticksPerTurntableRotation = lazySusanSubsystem.getTicksPerMotorRotation()*Constants.motorRotationsPerTurntableRotation;
        this.angleChangePerTick = 2*Math.PI/this.ticksPerTurntableRotation;        SmartDashboard.putNumber("Round to: ", 5);
        SmartDashboard.putNumber("Set default RPM: ", 0);
        // SmartDashboard.putNumber("Change angle: ", 0);
        addRequirements(shooterSubsystem);
        this.shooterSubsystem = shooterSubsystem;
        this.driverXbox = driverXbox;
        this.lazySusanSubsystem = lazySusanSubsystem;
    }

    @Override
    public void execute() {
        boolean hasTarget = entryHasTarget.getDouble(0.0) == 1.0;
        double x = entryX.getDouble(0.0);
        double y = entryY.getDouble(0.0);
        double area = entryArea.getDouble(0.0);

        input = driverXbox.getRightY();
        roundTo = SmartDashboard.getNumber("Round to: ", 5);
        currRPM = shooterSubsystem.getRPM();
        // deltaTheta = SmartDashboard.getNumber("Change angle: ", 0);
        // if(deltaTheta != 0) {
        //     // negative -> counterclockwise; positive -> clockwise
        //     currTicksGoal = (int)(deltaTheta/angleChangePerTick); 
        //     lazySusanSubsystem.getLazySusanEncoder().setPosition(0);
        // }
        // SmartDashboard.putNumber("Change angle: ", 0);

        // post to smart dashboard periodically
        SmartDashboard.putBoolean("LimelightHasTarget", hasTarget);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        // new math for static limelight shooting
        double tempDist = getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        double tempRPM = metersToRPM(tempDist * Constants.metersToFeet);


        //old math
        //double distance = Constants.hubHeight - Constants.limelightHeight;
        //double turretControl = turretControlConstant*x;
        //distance /= Math.tan(Constants.azimuthAngle1 + y); // canculate the current distance from the hub

        //double shootingAngle = (90-(Math.tan(distance/0.61)))*2; // variance allowed for x
        //SmartDashboard.putNumber("Current Distance", distance);
        //SmartDashboard.putNumber("ShootingAngle horizontal variance", shootingAngle);

/* // test to see if a shot can be taken
        if (shooterSubsystem.getRPM() == distance) {} // current RPM is correct for current distance
            if (Math.abs(x) <= shootingAngle) // To see if x variance will still allow for the ball to score
                SmartDashboard.putBoolean("Ready to fire", true);
            else {
                SmartDashboard.putBoolean("Ready to fire", false);
                if (x > 0)
                    //turretMotor.setSpeed(turretMotor.get()-turretControl) // when we eventually add a turret motor
                else if (x < 0)
                    //turretMotor.setSpeed(turretMotor.get()+turretControl)
        }
*/
        // double speed = driverXbox.getRightTriggerAxis();
        // shooterSubsystem.setShooterSpeed(speed);
        CANSparkMax temp = lazySusanSubsystem.getLazySusanMotor();
        boolean buttonPressed = false;
        temp.setOpenLoopRampRate(0.5);
        if (driverXbox.getXButtonPressed()) {
            if (temp.getEncoder().getVelocity() > 0) {
                temp.set(0);
            } else {
                temp.set(0.2);
            }
            buttonPressed = true;
        }
        if (driverXbox.getBButtonPressed()) {
            if (temp.getEncoder().getVelocity() > 0) {
                temp.set(0);
            } else {
                temp.set(-0.2);
            }
            buttonPressed = true;
        }
        if(!buttonPressed)
            temp.set(temp.getEncoder().getVelocity() + diffConstLS*x);
        // double currLSPos = lazySusanSubsystem.getLazySusanEncoder().getPosition();
        // if((currTicksGoal < 0 && currLSPos <= currTicksGoal) || 
        //     (currTicksGoal > 0 && currLSPos >= currTicksGoal)) {
        //     currTicksGoal = 0;
        //     lazySusanSubsystem.getLazySusanEncoder().setPosition(0);
        // } else {
        //     temp.set(diffConstLS * (currTicksGoal - currLSPos));
        // }
// //else if(hasTarget) {
//     if (driverXbox.getYButtonPressed()) {
//         setRPM(tempRPM);
//     }
        if (input > 0) {
            setRPM(input * (maxRPM - minRPM) + minRPM); 
        } else if(hasTarget) {
            if (driverXbox.getYButtonPressed()) {
                 setRPM(tempRPM);
            } 
        } else {
            setRPM(SmartDashboard.getNumber("Set default RPM: ", 0));
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
        return (h2 - h1) / Math.tan(a1 + a2);
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

    @Override
    public boolean isFinished() {
        return false;
    }
}