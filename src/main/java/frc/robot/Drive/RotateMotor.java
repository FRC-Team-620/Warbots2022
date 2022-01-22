package frc.robot.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class RotateMotor extends CommandBase {
    Drivetrain drivetrain;
    XboxController driverXbox;

    public RotateMotor(Drivetrain drivetrain, XboxController driverXbox) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.driverXbox = driverXbox;  
    }
    
    @Override
    public void execute() {
        motorRotate(2, 0.3, 3);
        /*if (driverXbox.getLeftBumperPressed()) {
            motorRotate(2, 0.3, 3);
            System.out.println("Left");
            //Do Stuff
        }
        if (driverXbox.getRightBumperPressed()) {
            motorRotate(4, 0.3, 3);
            System.out.println("Right");
            //Do Stuff
        }*/
    }
    @Override
    public boolean isFinished() {
        return false;
    }
    public void motorRotate(int idx, double v, int rotations) {
        idx = (idx-1)%4+1;
        int countsPerRev = 42;
        CANSparkMax mtr;
        switch(idx) {
          case 1:
            mtr = drivetrain.getMotor(1); //leftFrontMotor
            break;
          case 2:
            mtr = drivetrain.getMotor(2); //leftBackMotor
            break;
          case 3:
            mtr = drivetrain.getMotor(3); //rightFrontMotor
            break;
          default:
            mtr = drivetrain.getMotor(4); //rightBackMotor
        }
        RelativeEncoder enc = mtr.getEncoder();
        mtr.set(v);
        while(countsPerRev*enc.getPosition() < rotations) {
          continue;
        }
        mtr.set(0);
   }
}


