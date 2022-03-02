// package frc.robot.Climber;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class WindUpWinch extends CommandBase {
//     protected ClimberSubsystem climberSubsystem;
//     protected double counts;

//     public WindUpWinch(ClimberSubsystem climberSubsystem, double rotations) {
//         addRequirements(climberSubsystem);
//         this.climberSubsystem = climberSubsystem;
//         this.counts = rotations*climberSubsystem.getWinchMotor().getEncoder().getCountsPerRevolution();
//     }

//     @Override
//     public void initialize() {
//         climberSubsystem.getWinchMotor().getEncoder().setPosition(0);
//         System.out.println("Winch begins wind up");
//         climberSubsystem.getWinchMotor().set(1);
//     }

//     @Override
//     public void execute() {
//         System.out.println("Winch is winding");
//     }

//     @Override
//     public void end(boolean interrupted) {
//         climberSubsystem.getWinchMotor().set(0);
//     }

//     @Override
//     public boolean isFinished() {//climberSubsystem.getWinchMotor().getEncoder().getPosition() <= -counts
//         return climberSubsystem.getWinchMotor().getEncoder().getPosition() <= -counts;
//     }
    

// }
