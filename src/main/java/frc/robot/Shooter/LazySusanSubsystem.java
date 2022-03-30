package frc.robot.Shooter;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.function.Supplier;

import javax.crypto.interfaces.PBEKey;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class LazySusanSubsystem extends SubsystemBase {
    protected SimableCANSparkMax lazySusan;
    protected RelativeEncoder encoder;
    private PIDController lazySusanPID;
    private Rotation2d turretRotation;
    private Rotation2d desiredRotation;
    private final Supplier<Pose2d> robotBase;
    private boolean isGyroLocking;
    private final double countToDegreesFactor = (1.0/12.0) * (22.0/156.0) * 360.0;
    private double modSpeed = 1;
    private final double kP = 0.060000, kI = 0.003000, kD = 0;//KI0.00004 TODO: Tune PID Loop
    private boolean isCal;
    public final double lowLimit = -180.0 / countToDegreesFactor, highLimit = 180.0 / countToDegreesFactor;//Left 45.690002 Right -45.356651 AbsoluteMaxRange 90
    // private double turntableThresh = 35;

    public LazySusanSubsystem(Supplier<Pose2d> robotBase) {
        this.isGyroLocking = false;
        this.robotBase = robotBase;
        lazySusan = new SimableCANSparkMax(Constants.lazySusanID, MotorType.kBrushless);

        lazySusan.restoreFactoryDefaults();
        encoder = lazySusan.getEncoder();
        IdleMode mode = IdleMode.kBrake; // brakes
        lazySusan.setIdleMode(mode);

        lazySusan.setSmartCurrentLimit(35);

        //lazySusan.setInverted(true);

        lazySusanPID = new PIDController(kP, kI, kD);
        lazySusanPID.setTolerance(10);
        //lazySusanPID.setIntegratorRange(-10, 10);

        turretRotation = new Rotation2d();
        desiredRotation = turretRotation;
        isCal = false;
        SmartDashboard.putData(lazySusanPID);
    }

    @Override
    public void periodic() {
        
        turretRotation = Rotation2d.fromDegrees(lazySusan.getEncoder().getPosition() * countToDegreesFactor);

        if (isGyroLocking) {
            Rotation2d stablizedLocation = desiredRotation.minus(robotBase.get().getRotation());
            lazySusanPID.setSetpoint(MathUtil.clamp(stablizedLocation.getDegrees()/countToDegreesFactor, lowLimit, highLimit));
        } else {
            lazySusanPID.setSetpoint(MathUtil.clamp(desiredRotation.getDegrees()/countToDegreesFactor, lowLimit, highLimit));
        }

        
        lazySusanPID.calculate(lazySusan.getEncoder().getPosition());

        double lazySusanOutput = lazySusanPID.calculate(lazySusan.getEncoder().getPosition());
        double PIDOutput = MathUtil.clamp(lazySusanOutput, -1, 1);
        
        if (lazySusan.getEncoder().getPosition() > highLimit+5) {
            PIDOutput = MathUtil.clamp(PIDOutput, -1, 0);
        }
        if (lazySusan.getEncoder().getPosition() < lowLimit-5) {
            PIDOutput = MathUtil.clamp(PIDOutput, 0, 1);
        }

        lazySusan.set(PIDOutput * modSpeed);


        SmartDashboard.putNumber("TurretPos", lazySusan.getEncoder().getPosition());
        SmartDashboard.putNumber("LazySusanMotorPercentage", lazySusan.get());
        SmartDashboard.putBoolean("LazySusanSwitchTriggered", isCal);

        

        

    }

    // private void setTurretPosition(double x) {
    //     desiredRotation = x;
    // }
    
    public double getModSpeed() {
        return modSpeed;
    }

    public void setModSpeed(double mS) {
        modSpeed = mS;
    }


    public boolean getIsGyroLocking() {
        return isGyroLocking;
    }

    public void setIsGyroLocking(boolean iGL) {
        isGyroLocking = iGL;
    }

    public boolean getIsCal() {
        return isCal;
    }
    public void setIsCal(boolean iS) {
        isCal = iS;
    }



    public Rotation2d getRotation() {
        return turretRotation;
    }

    public Rotation2d getDesiredDegrees() {
        return desiredRotation;
    }
    
    public double getRawSetpoint() {
        return lazySusanPID.getSetpoint() * countToDegreesFactor;
    }


    public void setTurretPositionDegrees(Rotation2d rot) {
        desiredRotation =rot;
        // this.setTurretPosition(degrees/countToDegreesFactor);
    }


    public void updateRotation(double encoderCounts) {

    }


    public boolean atTurretPosition() {
        return lazySusanPID.atSetpoint();
    }

    public void setEncoderPosition(double p) {
        encoder.setPosition(p);
    }
    public void setHomePosition() {
        setIsGyroLocking(false);
        double limitPos = -45;
        setEncoderPosition(limitPos);
        setTurretPositionDegrees(Rotation2d.fromDegrees(limitPos*countToDegreesFactor));
        setIsCal(true);
    }

    public double getTicksPerMotorRotation() {
        return encoder.getCountsPerRevolution();
    }

    public double getDrawnCurrentAmps() {
        if (RobotBase.isSimulation()) {
            return this.simlazySusan.getCurrentDrawAmps();
        }
        return this.lazySusan.getOutputCurrent();
    }

    /**
     * Simulation Code
     */
    private DCMotorSim simlazySusan;
    private RevEncoderSimWrapper simEncoder;
    public Rotation2d simTurrentRotation;
    private boolean simInit = false;

    private void initSim() {
        simTurrentRotation = new Rotation2d();
        simlazySusan = new DCMotorSim(DCMotor.getNeo550(1), Constants.kSimTurntableGearRatio,
                Constants.kSimTurntableInertia); // TODO: add gear ratio
        simEncoder = RevEncoderSimWrapper.create(this.lazySusan);
    }

    @Override
    public void simulationPeriodic() {
        if (!simInit) {
            initSim();
            simInit = true;
        }
        simTurrentRotation = turretRotation;
        simlazySusan.setInputVoltage(lazySusan.get() * RobotController.getInputVoltage());
        simlazySusan.update(Constants.kSimUpdateTime);
        simEncoder.setVelocity(simlazySusan.getAngularVelocityRPM());
        simEncoder.setDistance(simlazySusan.getAngularPositionRotations()*180);
        // TODO: Remove magic number 5 that represents the first gear reduction
        SmartDashboard.putNumber("Turntable Velocity", encoder.getVelocity());
        SmartDashboard.putNumber("Turntable ticks", encoder.getPosition());
        SmartDashboard.putNumber("Turntable Set", this.lazySusanPID.getSetpoint());
    }
}
