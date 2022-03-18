package frc.robot.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualAiming extends CommandBase {
    LazySusanSubsystem lazySusanSubsystem;
    XboxController operatorXbox;
    RelativeEncoder lazySusanEnc;
    CANSparkMax lazySusanMotor;
    double inputOpLeft = 0;
    protected NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public ManualAiming(LazySusanSubsystem lazySusanSubsystem, XboxController operatorXbox) {
        this.lazySusanSubsystem = lazySusanSubsystem;
        this.operatorXbox = operatorXbox;
        lazySusanEnc = lazySusanSubsystem.getLazySusanEncoder();
        lazySusanMotor = lazySusanSubsystem.getLazySusanMotor();
        addRequirements(lazySusanSubsystem);
    }

    @Override
    public void execute() {
        inputOpLeft = operatorXbox.getLeftX();
        if (Math.abs(inputOpLeft) > 0) {
            table.getEntry("ledMode").setNumber(1);
            // limelight.setLEDMode(LedMode.OFF);
            double speed = 0;
            if (ShooterMath.inBounds(speed > 0, lazySusanEnc.getPosition())) {
                speed = -inputOpLeft/1.6;
            }
            
            // lazySusanMotor.set(ShooterMath.inBounds(-inputOpLeft > 0, lazySusanEnc.getPosition())
            //     ? -inputOpLeft/1.6 : 0);

            // System.out.println("Speed Man: "+ speed);
            // System.out.println("Curr Enc Pos: " + lazySusanEnc.getPosition());
        } else {
            lazySusanMotor.set(0);
        }
        if(operatorXbox.getRightBumper()) {
            lazySusanEnc.setPosition(0);
        }
        System.out.println("LAZY POS: " + lazySusanEnc.getPosition());
    }
}
