package frc.robot.Shooter;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

public class AutoAimingAndSpinningUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;

    
    protected NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    protected NetworkTableEntry entryHasTarget = table.getEntry("tv");
    protected NetworkTableEntry entryX = table.getEntry("tx");
    protected NetworkTableEntry entryY = table.getEntry("ty");
    protected NetworkTableEntry entryArea = table.getEntry("ta");
    
    // turntable
    protected double ticksPerTurntableRotation,angleChangePerTick;
    protected double  currTicksGoal = 0;
    protected double inputOpRight = 0;
    // shooter
    protected double rpm, minRPM = 0,
            currentSpeed = 0, acceleration, 
            currRPM;
    protected CANSparkMax lazySusanMotor;
    protected RelativeEncoder lazySusanEnc;
    public AutoAimingAndSpinningUp(ShooterSubsystem shooterSubsystem, LazySusanSubsystem lazySusanSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        lazySusanMotor = lazySusanSubsystem.getLazySusanMotor();
        lazySusanEnc = lazySusanSubsystem.getLazySusanEncoder();
    }
    public double getDistanceInMeters(double a1, double a2, double h1, double h2) {
        return (h2 - h1) / Math.tan((a1 + a2) * (Constants.degreesToRadians));
    }

    public double metersToRPM(double meters) {
        double distanceInFeet = Constants.metersToFeet * meters;
        System.out.println("Distance In Meters " + meters);
        return 117.3708 * distanceInFeet + 1632.61502;
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
    public void execute() {
        boolean hasTarget = entryHasTarget.getDouble(0.0) == 1.0;
        double x = entryX.getDouble(0.0);
        double y = entryY.getDouble(0.0);
        table.getEntry("ledMode").setNumber(3);
        double speeeeed = -x*Constants.diffConstLS; // this is lazy susan turntable speed
        double tempDist = getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        double tempRPM = metersToRPM(tempDist);
        // Making sure it's within the provided threshholds (important that you don't use absolute 
        // value -- don't obliterate the sign)
        if ((lazySusanEnc.getPosition() <= -Constants.turntableThresh && speeeeed < 0)
            || (lazySusanEnc.getPosition() >= Constants.turntableThresh && speeeeed > 0)) {
            speeeeed = 0;
        }
        lazySusanMotor.set(speeeeed);
        if(hasTarget) {
            setRPM(tempRPM);
        }
        acceleration = Constants.diffConstShooter * (getRPM() - currRPM);
        setShooterSpeedAndUpdate(currentSpeed + acceleration);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
