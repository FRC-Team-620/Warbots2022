package frc.robot.Util;

import com.revrobotics.CANSparkMax;

public class SparkPIDDrivewrapper extends CANSparkMax {
    private ControlType ctl;

    /**
     * Wraper class to override the Set method to use the pid contollers
     * SetReference. This is usefull when using classes like DifferentalDrive from
     * WPIlib where they only use the setmethod to control motor controlers. This
     * class allows you the override that behavior and use PID control with
     * differental drive.
     * 
     * @param deviceId the CANid of the spark max
     * @param type     Type of motor connected to the speed controler
     * @param ctl      The control mode to be used when set is called.
     */
    public SparkPIDDrivewrapper(int deviceId, MotorType type, ControlType ctl) {
        super(deviceId, type);
        this.ctl = ctl;
    }

    /**
     * Set the Control type to be used when the set method is called.
     * 
     * @param ctl
     */
    public void setPIDControlType(ControlType ctl) {
        this.ctl = ctl;
    }

    public ControlType getPIDControlType() {
        return this.ctl;
    }

    @Override
    public void set(double speed) {
        super.getPIDController().setReference(speed, this.ctl);
    }

    /**
     * The orignal behavior of the set method on the spark max.
     * 
     * @param speed the percent utilization to run the motor at.
     */
    public void setRaw(double speed) {
        super.set(speed);
    }

}
