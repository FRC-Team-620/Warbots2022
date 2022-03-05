package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Presets {
    public static final Map<String, List<Double>> presets = new HashMap<>() {{
        //List(double speed, double rotation, double rampRate)
        put("Default", List.of(Constants.speedHigh, Constants.rotationHigh, Constants.rampRate));
        put("Corbin", List.of(1.0, -0.75, 0.0));
        put("AJ", List.of(1.0, -0.75, 0.0));
    }};
}