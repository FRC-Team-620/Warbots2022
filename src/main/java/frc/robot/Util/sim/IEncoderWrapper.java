package frc.robot.Util.sim;

public interface IEncoderWrapper
{
    void setDistance(double distance);

    void setVelocity(double velocity);

    double getPosition();
}