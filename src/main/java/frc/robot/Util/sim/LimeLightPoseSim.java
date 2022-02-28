package frc.robot.Util.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLightPoseSim {
    private LimeLightSim sim;
    private Pose2d target, pos;
    private double height, angle;

    public LimeLightPoseSim(LimeLightSim sim, Pose2d target, double height, double targetHeight, double angle) {
        this.sim = sim;
        this.target = target;
        this.height = targetHeight - height;
        this.angle = angle;

    }

    public void setPosition(Pose2d pos) {
        this.pos = pos;
    }

    public void update(double timeStep) {
        // target.g
        var yangle = Math.atan2(height, target.getTranslation().getDistance(pos.getTranslation())) - Math.toRadians( this.angle);
        var deltap = target.minus(pos);
        var xangle = Math.atan2(deltap.getY(), deltap.getX());
        SmartDashboard.putNumber("X angle", Math.toDegrees(xangle));
        SmartDashboard.putNumber("Y Angle", Math.toDegrees(yangle));
        //If angle is less than preset angle it needds to be neg

    }

}
