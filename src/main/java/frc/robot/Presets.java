package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Presets {
    public static final Map<String, List<Double>> presets = new HashMap<>() {{
        //List(double speed, double rotation, double rampRate)
        put("Default", List.of(Constants.speed, Constants.rotation, Constants.rampRate));
    }};
}