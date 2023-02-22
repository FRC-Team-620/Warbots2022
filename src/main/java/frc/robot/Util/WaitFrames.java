package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitFrames extends CommandBase {
    protected int frames, endFrame;

    public WaitFrames(int endFrame) {
        this.endFrame = endFrame;
    }

    @Override
    public void initialize() {
        this.frames = 0;
    }

    @Override
    public void execute() {
        frames++;
    }

    @Override
    public boolean isFinished() {
        return this.frames >= this.endFrame;
    }

}
