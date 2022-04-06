package frc.robot.Shooter;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FiringPins extends SubsystemBase {
    protected CANSparkMax loader, extendedLoader;
    protected Solenoid firingPinsSolenoid, extension;
    // protected boolean isClimbing;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a
     * parameter. The device will be automatically initialized with default
     * parameters.
     */
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.i2cColorSensorPort);

    private final ColorMatch m_colorMatcher = new ColorMatch();

    private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    // private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
    private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    // private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

    public FiringPins() {
        firingPinsSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        firingPinsSolenoid.set(false);

        // m_colorSensor.configureProximitySensorLED(LEDPulseFrequency.kFreq100kHz,
        // LEDCurrent.kPulse125mA, 32);
        m_colorMatcher.addColorMatch(kBlueTarget);
        // m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        // m_colorMatcher.addColorMatch(kYellowTarget);
    }

    public Color detectedColor() {
        return m_colorSensor.getColor();
    }

    public double getProximity() {
        return this.m_colorSensor.getProximity();
    }

    public ColorMatchResult colorMatch() {
        return m_colorMatcher.matchClosestColor(this.detectedColor());
    }

    public Color matchedColor() {
        return this.colorMatch().color;
    }

    public double blueValue() {
        return this.m_colorSensor.getBlue();
    }

    public double redValue() {
        return this.m_colorSensor.getRed();
    }

    public boolean hasColor() {
        double blackLimit = 10;
        boolean proxTriggered = this.m_colorSensor.getProximity() >= Constants.minColorSensorProximity;
        boolean colorTrigered = blueValue() < blackLimit && redValue() < blackLimit;
        return proxTriggered && colorTrigered;
    }

    public void extendFiringPinsSolenoid() {
        firingPinsSolenoid.set(true);
    }

    public void retractFiringPinsSolenoid() {
        firingPinsSolenoid.set(false);
    }
}