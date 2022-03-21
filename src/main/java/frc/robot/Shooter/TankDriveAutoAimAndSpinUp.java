package frc.robot.Shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Drive.Drivetrain;

public class TankDriveAutoAimAndSpinUp extends CommandBase {
    protected ShooterSubsystem shooterSubsystem;
    protected Drivetrain drivetrain;
    
    protected NetworkTable table;
    protected NetworkTableEntry entryHasTarget;
    protected NetworkTableEntry entryX;
    protected NetworkTableEntry entryY;
    protected NetworkTableEntry entryArea;
    
    // turntable
    protected double inputOpRight = 0;
    protected int frames = 0;
    protected double speed = 0;
    // shooter
    protected double rpm, minRPM = 0,
            currentSpeed = 0, acceleration, 
            currRPM, maxRPM = 3000;
    protected XboxController operatorXbox;
    // auto
    protected boolean isAuto;
    protected boolean finished;

    public TankDriveAutoAimAndSpinUp(ShooterSubsystem shooterSubsystem, Drivetrain drivetrain, boolean isAuto, XboxController operatorXbox) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        entryHasTarget = table.getEntry("tv");
        entryX = table.getEntry("tx");
        entryY = table.getEntry("ty");
        entryArea = table.getEntry("ta");
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem, drivetrain);
        this.drivetrain = drivetrain;
        this.isAuto = isAuto;
        this.operatorXbox = operatorXbox;
    }

    @Override
    public void initialize() {
        this.frames = 0;
        table.getEntry("ledMode").setNumber(3);
    }

    // public void setShooterSpeedAndUpdate(double speed) {
    //     shooterSubsystem.setSpeed(speed);
    //     currentSpeed = speed;
    // }

    // public void setRPM(double rpm) {
    //     this.rpm = rpm;
    // }
    // public double getRPM() {
    //     return this.rpm;
    // }

    @Override
    public void execute() {
        //finished = false;
        boolean hasTarget = entryHasTarget.getDouble(0.0) == 1.0;
        double x = entryX.getDouble(0.0);
        double y = entryY.getDouble(0.0);
        
        speed = -(x-Constants.leftBias)*Constants.diffConstTankDriveAim;
        
         // this is lazy susan turntable speed
        // boolean hasTarget = shooterSubsystem.limeLight.hasTarget(); //Sim:
        // double x = shooterSubsystem.limeLight.getOffsetX();
        // double y = shooterSubsystem.limeLight.getOffsetY();
        // shooterSubsystem.limeLight.setLEDMode(LedMode.ON);
        // double speeeeed = -x*Constants.diffConstLS; // this is speed

        double tempDist = ShooterMath.getDistanceInMeters(Constants.azimuthAngle1, y, Constants.limelightHeight, Constants.hubHeight);
        double tempRPM = ShooterMath.metersToRPM(tempDist);
        // Making sure it's within the provided threshholds (important that you don't use absolute 
        // value -- don't obliterate the sign)
        // if (!ShooterMath.inBounds(speed > 0, this.drivetrain)) {
        //     speed = 0;
        // }
        this.drivetrain.tankDriveSet(-speed, speed);

        if(hasTarget) {
            System.out.println(tempRPM);
            // shooterSubsystem.setTargetRPM(tempRPM);
            shooterSubsystem.setTargetRPM(tempRPM);
            //System.out.println(shooterSubsystem.getTargetRPM());
        } else {
            // shooterSubsystem.setTargetRPM(0);
            shooterSubsystem.setTargetRPM(tempRPM);
        }
        System.out.println("hasTarget: " + hasTarget);
        // if(getRPM() > this.maxRPM)
        //     setRPM(this.maxRPM);
        //shooterSubsystem.setTargetRPM(Math.min(shooterSubsystem.getRPM(), this.maxRPM));
        //System.out.println(shooterSubsystem.getTargetRPM());
        System.out.println("RPM: " + shooterSubsystem.getRPM());
        acceleration = Constants.diffConstShooter * (shooterSubsystem.getTargetRPM() - shooterSubsystem.getRPM());
        shooterSubsystem.setShooterSpeedAndUpdate(shooterSubsystem.getCurrentSpeed() + acceleration);
        if (!(isAuto) && shooterSubsystem.getTargetRPM() > (1-Constants.shooterVibrationTolerance)*shooterSubsystem.getRPM()
            && shooterSubsystem.getTargetRPM() < (1+Constants.shooterVibrationTolerance)*shooterSubsystem.getRPM()) {
            operatorXbox.setRumble(RumbleType.kLeftRumble, 0.5);
            operatorXbox.setRumble(RumbleType.kRightRumble, 0.5);
        } else {
            operatorXbox.setRumble(RumbleType.kLeftRumble, 0);
            operatorXbox.setRumble(RumbleType.kRightRumble, 0);
        }
        
        System.out.println("ShooterSpeed: " + shooterSubsystem.getCurrentSpeed() + acceleration);
        //System.out.println("Frames: " + frames);
        
        // if (isAuto) {
        //     finished = true;
        // } 
        // else {
        //     frames++;
        // }
        if (isAuto) {
            frames++;
        }
    }

    @Override
    public void end(boolean interrupt) {
        //System.out.println("HERE!!!");
        this.drivetrain.tankDriveSet(0, 0);
        shooterSubsystem.stopMotors();
        table.getEntry("ledMode").setNumber(1);
        operatorXbox.setRumble(RumbleType.kLeftRumble, 0);
        operatorXbox.setRumble(RumbleType.kRightRumble, 0);
    }
        
    @Override
    public boolean isFinished() {
        // if (frames > 750) {
        //     return true;
        // } else if(finished) {
        //     return false;
        // }
        return isAuto && frames > 750;
    }
}
