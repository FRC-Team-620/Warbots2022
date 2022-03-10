package frc.robot.Shooter;

import java.text.DecimalFormat;
import java.util.List;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.LimeLight;
import frc.robot.Util.sim.LimeLightPoseSim;
import frc.robot.Util.sim.LimeLightSim;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class ShooterSubsystem extends SubsystemBase {
    private final SimableCANSparkMax rightShooterMotor, leftShooterMotor;
    private final MotorControllerGroup shooterMotors;
    protected final RelativeEncoder encoder;
    protected final DecimalFormat decFormat = new DecimalFormat("#.#");
    public LimeLight limeLight;
    public LimeLightPoseSim possim;

    //Sim
    FlywheelSim simFlywheel;
    RevEncoderSimWrapper leftencsim,rightencsim;
    LimeLightSim simLimeLight;

    public ShooterSubsystem() {
        rightShooterMotor = new SimableCANSparkMax(Constants.rightShooterMotorID, MotorType.kBrushless);
        leftShooterMotor = new SimableCANSparkMax(Constants.leftShooterMotorID, MotorType.kBrushless);
        shooterMotors = new MotorControllerGroup(rightShooterMotor, leftShooterMotor);
        encoder = rightShooterMotor.getEncoder();
        
        for (SimableCANSparkMax canSparkMax : List.of(rightShooterMotor, leftShooterMotor)) {
            canSparkMax.restoreFactoryDefaults();
            canSparkMax.setIdleMode(IdleMode.kCoast);
            canSparkMax.setSmartCurrentLimit(45);
        }
        
        limeLight = new LimeLight();
        if(RobotBase.isSimulation()){
            initSim();
        }
    }

    private void initSim(){
        simFlywheel = new FlywheelSim(DCMotor.getNEO(1), Constants.shooterGearRatio, Constants.kSimShooterInertia); //TODO replace with sim const

        this.leftencsim = RevEncoderSimWrapper.create(this.leftShooterMotor);
        this.rightencsim = RevEncoderSimWrapper.create(this.rightShooterMotor);
        this.simLimeLight = new LimeLightSim(this.limeLight);
        var hubpos =  new Pose2d(7.940, 4.08, new Rotation2d());
        this.possim = new LimeLightPoseSim(simLimeLight, hubpos, Constants.limelightHeight, Constants.hubHeight, Constants.azimuthAngle1);
    }

    public double getTicksPerMotorRotation() {
        return encoder.getCountsPerRevolution();
    }

    public double getRPM() {
        return encoder.getVelocity();
    }

    public void stopMotors() {
        shooterMotors.stopMotor();
    }
    
    public long getTotalWheelRotations() {
        return (long) encoder.getPosition(); // The conversion factor was previously set
    }

    public double getSpeed() {
        return shooterMotors.get();
    }

    public void setSpeed(double speed) {
        SmartDashboard.putNumber("shooter setpoint speed", speed);
        shooterMotors.set(speed);
    }

    public double getSimRPM(){
        return simFlywheel.getAngularVelocityRPM();
    }

    @Override
    public void simulationPeriodic() {
        simFlywheel.setInputVoltage(rightShooterMotor.get() * RobotController.getInputVoltage());
        simFlywheel.update(Constants.kSimUpdateTime);
        SmartDashboard.putNumber("Right Flywheel Sim", simFlywheel.getAngularVelocityRPM());
        rightencsim.setVelocity(simFlywheel.getAngularVelocityRPM());
        leftencsim.setVelocity(simFlywheel.getAngularVelocityRPM());
        
        SmartDashboard.putNumber("Right Flywheel motor", getRPM());
    }

    public double getDrawnCurrentAmps(){
        if(RobotBase.isSimulation()){
            return this.simFlywheel.getCurrentDrawAmps()*2;
        }
        return this.rightShooterMotor.getOutputCurrent() + this.leftShooterMotor.getOutputCurrent();
    }
}