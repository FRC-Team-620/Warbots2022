package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

public class RotateMotor extends CommandBase {
    Drivetrain drivetrain;
    XboxController driverXbox;
    public RotateMotor(Drivetrain drivetrain, XboxController driverXbox) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.driverXbox = driverXbox;  
    }
    public void execute() {
        if (driverXbox.getLeftBumperPressed()) {
            //Do Stuff
        }
        if (driverXbox.getRightBumperPressed()) {
            //Do Stuff
          }
    }

}

// public void motorRotate(int idx, double v, int rotations) {
//     idx = (idx-1)%4+1;
//     int countsPerRev = 42;
//     CANSparkMax mtr;
//     switch(idx) {
//       case 1:
//         mtr = leftFrontMotor;
//         break;
//       case 2:
//         mtr = leftBackMotor;
//         break;
//       case 3:
//         mtr = rightFrontMotor;
//         break;
//       default:
//         mtr = rightBackMotor;
//     }
//     RelativeEncoder enc = mtr.getEncoder();
//     mtr.set(v);
//     while(countsPerRev*enc.getPosition() < rotations) {
//       continue;
//     }
//     mtr.set(0);
//   }
