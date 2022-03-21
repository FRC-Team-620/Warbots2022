package frc.robot.Util.sim;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimValue;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Util.LimeLight;

public class LimeLightSim {
    private final NetworkTableEntry e_tv, e_tx, e_ty, e_ta, e_tl;
    private final SimValue s_tv, s_tx, s_ty, s_ta, s_tl;
    //private SimDeviceSim simd;

    public LimeLightSim(LimeLight limelight) {
        NetworkTable ntable = limelight.getTable();
        SimDeviceSim simd = new SimDeviceSim("LimeLight", limelight.getId());
        System.out.println(limelight.getId());
        e_tv = ntable.getEntry("tv");
        s_tv = simd.getBoolean("Has Target");
        e_tx = ntable.getEntry("tx");
        s_tx = simd.getDouble("Offset X");
        e_ty = ntable.getEntry("ty");
        s_ty = simd.getDouble("Offset Y");
        e_ta = ntable.getEntry("ta");
        s_ta = simd.getDouble("Target Area");
        e_tl = ntable.getEntry("tl");
        s_tl = simd.getDouble("Pipeline Latancy");
        //this.simd = simd;
    }

    public void setHasTarget(boolean hastarget) {
        e_tv.setNumber(hastarget ? 1 : 0);
        s_tv.setValue(HALValue.makeBoolean(hastarget));
    }

    public void setOffsetY(double angle) {
        e_ty.setNumber(angle);
        s_ty.setValue(HALValue.makeDouble(angle));
    }

    public void setOffsetX(double angle) {
        // System.out.println(simd.enumerateValues().length);
        // System.out.println(simd.enumerateValues()[0]);

        e_tx.setNumber(angle);
        s_tx.setValue(HALValue.makeDouble(angle));
    }

    public void setArea(double area) {
        e_ta.setNumber(area);
        s_ta.setValue(HALValue.makeDouble(area));
    }

    public void setLatancy(double ms) {
        e_tl.setNumber(ms);
        s_tl.setValue(HALValue.makeDouble(ms));
    }

}
