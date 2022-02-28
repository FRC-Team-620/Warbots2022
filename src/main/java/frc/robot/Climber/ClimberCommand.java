package frc.robot.Climber;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimberCommand extends SequentialCommandGroup {
    protected ClimberSubsystem climberSubsystem;
    protected double encoderTicksPerWinchRotation = 77.5;

    public ClimberCommand(ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        addCommands(
            new WindDownWinch(climberSubsystem, encoderTicksPerWinchRotation), // unwind winch

            new RaiseArms(climberSubsystem), // extend piston/arm

            new WindUpWinch(climberSubsystem, encoderTicksPerWinchRotation/2), // rewind winch 1/2-way
            
            new LowerArms(climberSubsystem), // retract piston/arm

            new WindUpWinch(climberSubsystem, encoderTicksPerWinchRotation/2), // rewind winch the rest of the way

            new RaiseHooks(climberSubsystem), // latch hooks

            new WindDownWinch(climberSubsystem, encoderTicksPerWinchRotation), // unwind winch

            new RaiseArms(climberSubsystem), // extend piston/arms

            new WindUpWinch(climberSubsystem, encoderTicksPerWinchRotation/2), // rewind winch 1/2-way

            new LowerHooks(climberSubsystem), // unlatch hooks

            new LowerArms(climberSubsystem), // retract piston/arms

            new WindUpWinch(climberSubsystem, encoderTicksPerWinchRotation/2), // rewind winch rest of the way

            new RaiseHooks(climberSubsystem), // latch hooks second round

            new WindDownWinch(climberSubsystem, encoderTicksPerWinchRotation), // unwind winch

            new RaiseArms(climberSubsystem), // extend piston/arms

            new WindUpWinch(climberSubsystem, encoderTicksPerWinchRotation/2), // rewind winch 1/2-way

            new LowerHooks(climberSubsystem), // unlatch hooks

            new LowerArms(climberSubsystem), // retract piston/arms

            new WindUpWinch(climberSubsystem, encoderTicksPerWinchRotation/2) // rewind winch rest of the way

        );
    }

    @Override
    public void execute() {
        // if (driverXbox.getYButtonPressed()) {
        //     climberSubsystem.getArmsSolenoid().set(true);
            
        // }
        // Testing if it's working
        // climberSubsystem.climberSolenoid.set(driverXbox.getYButton());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
