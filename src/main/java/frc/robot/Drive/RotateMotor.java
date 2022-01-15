package frc.robot.Drive;

public class RotateMotor {
    
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
