package frc.robot.Util;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class SendableCANPIDController implements Sendable {
    private static int instances = 10;
    private SparkMaxPIDController pid;
    private double lastSetpoint = Double.NaN;
    private ControlType controlType;

    /**
     * A wraper class to allow Sparkmax pidloops to be tuned from within
     * smartdashboard or glass. Add this display with SmartDashboard.putData(new
     * SendableCANPIDController(args));
     * 
     * @param pid         PID loop to use
     * @param controlType Control type to be used when setpoint is set.
     */
    public SendableCANPIDController(SparkMaxPIDController pid, ControlType controlType) {
        this.pid = pid;
        this.controlType = controlType;
        SendableRegistry.addLW(this, "PIDController", instances++); // TODO: do a less janky solution to this.
    }

    /**
     * Sets the Reference for the PID loop. This method should only be used by
     * smartdashboards Sendable/Builder
     * 
     * @param setpoint Setpoint to hit
     */
    public void setSetpoint(double setpoint) {
        this.setSetpoint(setpoint, this.controlType);

    }

    /**
     * Sets the Reference for the PID loop including the control mode
     * 
     * @param setpoint    Setpoint to hit
     * @param controlType Control mode to use
     */
    public void setSetpoint(double setpoint, ControlType controlType) {
        this.lastSetpoint = setpoint;
        this.pid.setReference(setpoint, controlType);
    }

    /**
     * A hacky getSetpoint method that just returns the last setpoint set via this
     * class or smartdashboard. Does not Live track changes to the setpoint via the
     * Smarkmax controler.
     * 
     * @return returns the last set setpoint from this class.
     */
    public double getSetpoint() {
        // System.err.println("Get setpoint is not supported, returning last set
        // setpoint");
        return this.lastSetpoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this.pid::getP, this.pid::setP);
        builder.addDoubleProperty("i", this.pid::getI, this.pid::setI);
        builder.addDoubleProperty("d", this.pid::getD, this.pid::setD);
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);

    }

}
