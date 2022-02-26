package frc.robot.Util;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    private NetworkTable ntable;
    private final NetworkTableEntry e_tv, e_tx, e_ty, e_ta, e_tl, e_ledMode, e_pipeline, e_camMode, e_stream,
            e_snapshot, e_getpipe;

    private SimDevice m_simDevice;
    private SimBoolean s_tv;
    private SimDouble s_tx, s_ty, s_ta, s_tl;
    private static int index = 0;

    public LimeLight() {
        this(NetworkTableInstance.getDefault().getTable("limelight"));
    }

    public LimeLight(NetworkTable ntable) {
        // entryMap = new HashMap<>();
        this.ntable = ntable;
        e_tv = ntable.getEntry("tv");
        e_tx = ntable.getEntry("tx");
        e_ty = ntable.getEntry("ty");
        e_ta = ntable.getEntry("ta");
        e_tl = ntable.getEntry("tl");
        e_ledMode = ntable.getEntry("ledMode");
        e_pipeline = ntable.getEntry("pipeline");
        e_camMode = ntable.getEntry("camMode");
        e_stream = ntable.getEntry("stream");
        e_snapshot = ntable.getEntry("snapshot");
        e_getpipe = ntable.getEntry("getpipe");

        m_simDevice = SimDevice.create("LimeLight", index++);
        if (isSim()) {
            s_ty = m_simDevice.createDouble("Offset Y", Direction.kInput, 0.0);
            s_tx = m_simDevice.createDouble("Offset X", Direction.kInput, 0.0);
            s_ta = m_simDevice.createDouble("Target Area", Direction.kInput, 0.0);
            s_tl = m_simDevice.createDouble("Pipeline Latancy", Direction.kInput, 0.0);
            s_tv = m_simDevice.createBoolean("Has Target", Direction.kInput, false);
        }

    }

    private boolean isSim() {
        return m_simDevice != null;
    }

    public int getId() {
        return index;
    }

    public boolean hasTarget() {
        if (isSim()) {
            return s_tv.get();
        }
        return e_tv.getDouble(0.0) == 1;
    }

    public double getOffsetY() {
        if (isSim()) {
            return s_ty.get();
        }
        return e_ty.getDouble(0.0);
    }

    public double getOffsetX() {
        if (isSim()) {
            return s_tx.get();
        }
        return e_tx.getDouble(0.0);
    }

    public double getArea() {
        if (isSim()) {
            return s_ta.get();
        }
        return e_ta.getDouble(0.0);
    }

    // TODO: add get skew
    public double getLatency() {
        if (isSim()) {
            return s_tl.get();
        }
        return e_tl.getDouble(0.0);
    }

    public LedMode getLEDMode() {
        int modeid = (int) e_ledMode.getDouble(0.0);
        return LedMode.fromInt(modeid);
    }

    public int getSetPipline() {
        return (int) e_pipeline.getDouble(0.0);
    }

    public double getActivePipline() {
        return (int) e_getpipe.getDouble(0.0);
    }

    public boolean setLEDMode(LedMode mode) {
        return e_ledMode.setNumber(mode.getValue());
    }

    public boolean setPipeLine(int pipeline) {
        if (pipeline < 0 || pipeline > 9) { // Check that valid pipline is set
            throw new IllegalArgumentException("Pipline must be between 0 and 9");
        }
        return e_pipeline.setNumber(pipeline);

    }

    public boolean setCamMode(CamMode mode) {
        return e_camMode.setNumber(mode.getValue());
    }

    public boolean setStreamMode(StreamMode mode) {
        return e_stream.setNumber(mode.getValue());
    }

    public boolean enableSnapshots(boolean dosnapshot) {
        return e_snapshot.setNumber(dosnapshot ? 1 : 0);
    }

    public NetworkTable getTable() {
        return this.ntable;
    }

    public enum LedMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        private final int value;

        private LedMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }

        private static Map<Integer, LedMode> reverseLookup = Arrays.stream(LedMode.values())
                .collect(Collectors.toMap(LedMode::getValue, Function.identity()));

        public static LedMode fromInt(final int id) {
            return reverseLookup.getOrDefault(id, PIPELINE);
        }
    }

    public enum CamMode {
        Vision(0), DriverCamera(0);

        private final int value;

        private CamMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }

        private static Map<Integer, CamMode> reverseLookup = Arrays.stream(CamMode.values())
                .collect(Collectors.toMap(CamMode::getValue, Function.identity()));

        public static CamMode fromInt(final int id) {
            return reverseLookup.getOrDefault(id, Vision);
        }
    }

    public enum StreamMode {
        SideBySide(0), PictureInPictureMain(1), PictureInPictureSecondary(2);

        public final int value;

        private StreamMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return this.value;
        }

        private static Map<Integer, StreamMode> reverseLookup = Arrays.stream(StreamMode.values())
                .collect(Collectors.toMap(StreamMode::getValue, Function.identity()));

        public static StreamMode fromInt(final int id) {
            return reverseLookup.getOrDefault(id, SideBySide);
        }
    }

}