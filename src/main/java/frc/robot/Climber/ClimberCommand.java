package frc.robot.Climber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ClimberCommand extends SequentialCommandGroup {
    protected ClimberSubsystem climberSubsystem;

    protected int prevTime = -1;

    public ClimberCommand(ClimberSubsystem climberSubsystem) {
        addRequirements(climberSubsystem);
        this.climberSubsystem = climberSubsystem;
        addCommands(
            new WindDownWinch(climberSubsystem, 77.5),

            new RaiseArms(climberSubsystem),

            new WindUpWinch(climberSubsystem, 38.75),
            
            new LowerArms(climberSubsystem),

            new WindUpWinch(climberSubsystem, 38.75),

            new RaiseHooks(climberSubsystem),

            new WindDownWinch(climberSubsystem, 77.5),

            new RaiseArms(climberSubsystem),

            new WindUpWinch(climberSubsystem, 38.75),

            new LowerHooks(climberSubsystem),

            new LowerArms(climberSubsystem),

            new WindUpWinch(climberSubsystem, 38.75),

            new RaiseHooks(climberSubsystem),//second round

            new WindDownWinch(climberSubsystem, 77.5),

            new RaiseArms(climberSubsystem),

            new WindUpWinch(climberSubsystem, 38.75),

            new LowerHooks(climberSubsystem),

            new LowerArms(climberSubsystem),

            new WindUpWinch(climberSubsystem, 38.75)





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
