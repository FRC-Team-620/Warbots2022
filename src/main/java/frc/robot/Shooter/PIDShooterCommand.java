// package frc.robot.Shooter;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class PIDShooterCommand extends CommandBase {
//     PIDController controller;
//     ShooterSubsystem shooter;
//     private boolean powerDecel = true;
//     public PIDShooterCommand(ShooterSubsystem shooter) {
//         addRequirements(shooter);
//         this.shooter = shooter;
//         this.controller = new PIDController(0.005, 0.008, 0);
//         this.controller.setTolerance(10, 3); //Add a velocity component
//         SmartDashboard.putData(this.controller);
//         // this.controller.

//     }

//     @Override
//     public void execute() {
//         // TODO Auto-generated method stub
//         var output = this.controller.calculate(this.shooter.getSimRPM());
        
//         output = MathUtil.clamp(output,powerDecel ? -1: 0,1);
//         SmartDashboard.putNumber("Shooter pid out", output);
//         this.shooter.setSpeed(output);
        
//         super.execute();
//     }

//     @Override
//     public boolean isFinished() {
//         // return this.controller.atSetpoint()
//         return false;

//     }
// }
