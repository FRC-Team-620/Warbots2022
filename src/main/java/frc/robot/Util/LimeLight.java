package frc.robot.Util;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class LimeLight {
    private static NetworkTable limelightTable;
    private static NetworkTableEntry txEntry; // Horizontal offset to target
    private static NetworkTableEntry tyEntry; // Vertical offset to target
    private static NetworkTableEntry tvEntry; // Whether a target is found
    private static NetworkTableEntry ledModeEntry; // The state of the front-facing lights
    // private static NetworkTableEntry streamEntry; // The display mode of the camera

    public enum LedMode {
        OFF(1), ON(3);

        private final int value;

        private LedMode(int value) {
            this.value = value;
        }

        public int get() {
            return this.value;
        }

        // A map of the enum's integer values to the enum itself
        private static Map<Integer, LedMode> reverseLookup = Arrays.stream(LedMode.values())
            .collect(Collectors.toMap(LedMode::get, Function.identity()));

        public static LedMode fromInt(int number) {
            return reverseLookup.getOrDefault(number, OFF);
        }
    }

    public static void init() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        txEntry = limelightTable.getEntry("tx");
        tyEntry = limelightTable.getEntry("ty");
        tvEntry = limelightTable.getEntry("tv");
        ledModeEntry = limelightTable.getEntry("ledMode");
        // streamEntry = limelightTable.getEntry("stream");
        // streamEntry.setDouble(2); // Puts the LimeLight in secondary picture-in-picture mode
        setLedMode(LedMode.OFF); // Ensure that the LimeLight starts in off mode
    }

    public static LedMode getLedMode() {
        return LedMode.fromInt(ledModeEntry.getNumber(0).intValue());
    }

    public static void setLedMode(LedMode ledMode) {
        ledModeEntry.setNumber(ledMode.get());
    }

    public static double getTX() {
        return txEntry.getDouble(0);
    }

    public static double getTY() {
        return tyEntry.getDouble(0);
    }

    public static boolean hasTarget() {
        return tvEntry.getDouble(0) == 1;
    }

    // In meters
    public static double getDistance() {
        return (Constants.hubHeight-Constants.limelightHeight)/Math.tan(Math.toRadians(Constants.azimuthAngle1+getTY()));
    }

    public static boolean inRange() {
        return getDistance() < Constants.maxShootingDistance;
    }
}
