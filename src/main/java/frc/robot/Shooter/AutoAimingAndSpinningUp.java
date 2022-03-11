package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight.LedMode;

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
    protected int frames = 0;
    // shooter
    protected double rpm, minRPM = 0,
            currentSpeed = 0, acceleration, 
            currRPM, maxRPM = 3000;
    protected CANSparkMax lazySusanMotor;
    protected RelativeEncoder lazySusanEnc;
    // auto
    protected boolean isAuto;
    protected boolean finished;
    public AutoAimingAndSpinningUp(ShooterSubsystem shooterSubsystem, LazySusanSubsystem lazySusanSubsystem, boolean isAuto) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
        lazySusanMotor = lazySusanSubsystem.getLazySusanMotor();
        lazySusanEnc = lazySusanSubsystem.getLazySusanEncoder();
        this.isAuto = isAuto;
    }

    @Override
    public void initialize() {
        this.frames = 0;
    }

    public void setShooterSpeedAndUpdate(double speed) {
        shooterSubsystem.setSpeed(speed);
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
        finished = false;
        boolean hasTarget = entryHasTarget.getDouble(0.0) == 1.0;
        double x = entryX.getDouble(0.0);
        double y = entryY.getDouble(0.0);
        table.getEntry("ledMode").setNumber(3);
        double speed = -x*Constants.diffConstLS; // this is lazy susan turntable speed
        // boolean hasTarget = shooterSubsystem.limeLight.hasTarget(); //Sim:
        // double x = shooterSubsystem.limeLight.getOffsetX();
        // double y = shooterSubsystem.limeLight.getOffsetY();
        // shooterSubsystem.limeLight.setLEDMode(LedMode.ON);
        // double speeeeed = -x*Constants.diffConstLS; // this is speed

        double tempDist = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        double tempRPM = ShooterMath.metersToRPM(tempDist);
        // Making sure it's within the provided threshholds (important that you don't use absolute 
        // value -- don't obliterate the sign)
        if (!ShooterMath.inThreshold(speed > 0, lazySusanEnc.getPosition())) {
            speed = 0;
        }
        lazySusanMotor.set(speed);
        if(hasTarget) {
            setRPM(tempRPM);
        }
        // if(getRPM() > this.maxRPM)
        //     setRPM(this.maxRPM);
        setRPM(Math.min(getRPM(), this.maxRPM));
        acceleration = Constants.diffConstShooter * (getRPM() - currRPM);
        setShooterSpeedAndUpdate(currentSpeed + acceleration);
        //System.out.println("Frames: " + frames);
        frames++;
        if (isAuto) {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (frames > 750) {
            return true;
        } else if(finished) {
            return true;
        }
        return false;
        
    }

    
}
