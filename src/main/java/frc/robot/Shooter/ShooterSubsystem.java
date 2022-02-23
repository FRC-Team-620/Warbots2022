package frc.robot.Shooter;

import java.text.DecimalFormat;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SimableCANSparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.sim.RevEncoderSimWrapper;

public class ShooterSubsystem extends SubsystemBase {
    protected final SimableCANSparkMax leftShooterMotor, rightShooterMotor;
    protected final RelativeEncoder encoder;
    protected final DecimalFormat decFormat = new DecimalFormat("#.#");

    //Sim
    FlywheelSim simFlywheel;
    RevEncoderSimWrapper leftencsim,rightencsim;

    public ShooterSubsystem() {
        leftShooterMotor = new SimableCANSparkMax(Constants.leftShooterMotorID, MotorType.kBrushless);//TODO: Refactor name to follower
        rightShooterMotor = new SimableCANSparkMax(Constants.rightShooterMotorID, MotorType.kBrushless); 
        encoder = rightShooterMotor.getEncoder();

        leftShooterMotor.restoreFactoryDefaults();
        rightShooterMotor.restoreFactoryDefaults();

        IdleMode mode = IdleMode.kCoast; // brakes
        leftShooterMotor.setIdleMode(mode);
        rightShooterMotor.setIdleMode(mode);

        leftShooterMotor.follow(rightShooterMotor, true);
        // leftShooterMotor.setInverted(false);
        // rightShooterMotor.setInverted(false);
        if(RobotBase.isSimulation()){
            initSim();
        }
    }

    private void initSim(){
        simFlywheel = new FlywheelSim(DCMotor.getNEO(1), Constants.shooterGearRatio, Constants.kSimShooterInertia); //TODO replace with sim const

        this.leftencsim = RevEncoderSimWrapper.create(this.leftShooterMotor);
        this.rightencsim = RevEncoderSimWrapper.create(this.rightShooterMotor);
    }

    public double getTicksPerMotorRotation() {
        return encoder.getCountsPerRevolution();
    }
    public double getRPM() {
        return encoder.getVelocity(); //TODO: Why convert to string and then parse double. Use math round functions
    }
    public long getTotalWheelRotations() {
        return (long)encoder.getPosition(); // The conversion factor was previously set
    }
    // public double getRPM(long prevRotations, double secondsTimestep) {
    //     return (this.getTotalWheelRotations() - prevRotations) / (secondsTimestep/60);
    // }

    public void setShooterSpeed(double speed) {
        SmartDashboard.putNumber("shooter setpoint speed", speed);
        // System.out.println(speed);
        rightShooterMotor.set(speed);
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
        // simFlywheel.set
    }

    public double getDrawnCurrentAmps(){
        if(RobotBase.isSimulation()){
            return this.simFlywheel.getCurrentDrawAmps()*2;
        }
        return this.rightShooterMotor.getOutputCurrent() + this.leftShooterMotor.getOutputCurrent();
    }
}