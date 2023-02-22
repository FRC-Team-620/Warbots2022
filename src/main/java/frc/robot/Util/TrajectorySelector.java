package frc.robot.Util;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajectorySelector extends SendableChooser<Trajectory> {
    private Field2d field;
    private Trajectory part1;
    private Trajectory part2;
    private Trajectory part3;
    private Trajectory part4;


    /**
     * Class to automatically load and display Trajectory information. This class
     * will automatically scan a selected dir (shallow scan) and attempt to load
     * every file. The loaded trajectories will be displayed in a sendable chooser
     * on
     * smartdashboard. Field2ds can also be added so that displayed trajectories
     * automatically update with changes.
     * 
     * @param directory    Directory to load Trajectory json files from
     * @param defaultFirst Wether or not to set the first found file as the default
     *                     value. See getSelection()
     */
    public TrajectorySelector(Path directory) {
        this(directory, false);

    }

    public TrajectorySelector(Path directory, boolean defaultFirst) {
        super();
        this.loadTajectorys(directory, defaultFirst);
        SmartDashboard.putData(this);// Not sure if I should automatically add this to the dashboard or not.
    }

    /**
     * Loads trajectory files from a directory.
     * 
     * @param directory    Directory to load Trajectory json files from
     * @param defaultFirst Wether or not to set the first found file as the default
     *                     value. See getSelection()
     */
    private void loadTajectorys(Path directory, boolean defaultFirst) {
        var files = directory.toFile().listFiles();// May want to filter files that only end with wpilib.json
        for (File file : files) {
            if (!file.isFile()) {
                continue;
            }
            String name = file.getName();
            Trajectory traj;
            try {
                traj = TrajectoryUtil.fromPathweaverJson(file.toPath());
                if (defaultFirst) {
                    this.setDefaultOption("* " + name, traj);
                    defaultFirst = false;
                } else {
                    this.addOption("* " + name, traj);// Add star to paths imported from disk scan
                }
                if (name.equals("Part1")) {
                    part1 = traj;
                } else if (name.equals("Part2")) {
                    part2 = traj;
                } else if (name.equals("Part3")) {
                    part3 = traj;
                } else if (name.equals("Part4")) {
                    part4 = traj;
                }
            } catch (IOException e) {
                e.printStackTrace();
                DriverStation.reportError("Unable to open trajectory: " + file.getPath(), e.getStackTrace());
            }

        }

    }

    public Trajectory getPart1() {
        return part1;
    }
    public Trajectory getPart2() {
        return part2;
    }
    public Trajectory getPart3() {
        return part3;
    }
    public Trajectory getPart4() {
        return part4;
    }

    /**
     * Called when a selection on smardashboard is made.
     * 
     * @param data string of selected option
     */
    private void onSelection(String data) {
        if (this.field != null) {
            this.field.getObject("traj").setTrajectory(this.getSelected());
        }
    }

    /**
     * Links a field2d to this selector. This causes the Field to automaticly update
     * its displayed trajectory to match the current selection of this widget.
     * 
     * @param field field2d to link.
     */
    public void linkField(Field2d field) { // TODO: rename
        this.field = field;
        onSelection("");
    }

    @Override
    public void initSendable(NTSendableBuilder builder) {
        // Ignore this this is a supper janky way to detect when the selection is
        // changed. I should switch this to use Network tables EntryListener.
        // All this nonesne was to avoid needing to use refelction.
        Jank buildProxy = new Jank(builder, "selected", this::onSelection);
        super.initSendable(buildProxy);
    }

    private class Jank implements NTSendableBuilder {
        NTSendableBuilder builder;
        String interceptKey;
        Consumer<String> intercept, callback;

        public Jank(NTSendableBuilder builder, String interceptKey, Consumer<String> callback) {
            this.callback = callback;
            this.builder = builder;
            this.interceptKey = interceptKey;
        }

        public void proxy(String indata) {
            this.intercept.accept(indata);
            this.callback.accept(indata);

        }

        @Override
        public void addRawProperty(String key, Supplier<byte[]> getter, Consumer<byte[]> setter) {
            this.builder.addRawProperty(key, getter, setter);

        }

        @Override
        public void addBooleanProperty(String key, BooleanSupplier getter, BooleanConsumer setter) {
            this.builder.addBooleanProperty(key, getter, setter);

        }

        @Override
        public void addBooleanArrayProperty(String key, Supplier<boolean[]> getter, Consumer<boolean[]> setter) {
            this.builder.addBooleanArrayProperty(key, getter, setter);

        }

        @Override
        public void addStringArrayProperty(String key, Supplier<String[]> getter, Consumer<String[]> setter) {
            this.builder.addStringArrayProperty(key, getter, setter);

        }

        @Override
        public NetworkTable getTable() {
            return this.builder.getTable();
        }

        @Override
        public void setSmartDashboardType(String type) {
            this.builder.setSmartDashboardType(type);
        }

        @Override
        public void update() {
            this.builder.update();
        }

        @Override
        public void addStringProperty(String key, Supplier<String> getter, Consumer<String> setter) {
            if (this.interceptKey.equals(key)) {
                this.intercept = setter;
                this.builder.addStringProperty(key, getter, this::proxy);
                // this.intercept = null;
            } else {
                this.builder.addStringProperty(key, getter, setter);
            }

        }

        @Override
        public void setActuator(boolean value) {
            this.builder.setActuator(value);

        }

        @Override
        public void clearProperties() {
            this.clearProperties();
        }

        @Override
        public void addDoubleArrayProperty(String key, Supplier<double[]> getter, Consumer<double[]> setter) {
            this.builder.addDoubleArrayProperty(key, getter, setter);
        }

        @Override
        public NetworkTableEntry getEntry(String key) {
            return this.builder.getEntry(key);
        }

        @Override
        public void addDoubleProperty(String key, DoubleSupplier getter, DoubleConsumer setter) {
            this.builder.addDoubleProperty(key, getter, setter);
        }

        @Override
        public void addValueProperty(String key, Supplier<NetworkTableValue> getter,
                Consumer<NetworkTableValue> setter) {
            this.builder.addValueProperty(key, getter, setter);
        }

        @Override
        public boolean isPublished() {
            return this.builder.isPublished();
        }

        @Override
        public void setUpdateTable(Runnable func) {
            this.builder.setUpdateTable(func);
        }

        @Override
        public void setSafeState(Runnable func) {
            this.builder.setSafeState(func);
        }
    }
}
