package frc.robot.Loader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shooter.FiringPins;
// import frc.robot.Shooter.ShooterSubsystem;

public class SmartIntake extends CommandBase {
    protected Intake intake;
    protected FiringPins firingPins;

    // protected ShooterSubsystem shooterSubsystem;
    public SmartIntake(Intake intake, FiringPins firingPins) {
        this.intake = intake;
        this.firingPins = firingPins;
        // this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        // intake.enableInnerIntakeMotor();
        this.intake.extendIntakeArmsSolenoid();
        this.intake.enableIntakeArmsMotor();
    }

    @Override
    public void execute() {
        boolean hasTopBall = this.firingPins.hasColor();
        boolean hasBottomBall = this.intake.getIntakeSwitch();
        System.out.println("COLOR: " + this.firingPins.detectedColor());
        System.out.println("PROX: " + this.firingPins.getProximity());
        System.out.println("BLUE: " + this.firingPins.blueValue());
        System.out.println("RED: " + this.firingPins.redValue());
        System.out.println("BALL 1: " + hasTopBall);
        System.out.println("BALL 2: " + hasBottomBall);
        if (hasTopBall && hasBottomBall) {
            System.out.println("<<<STOP>>>");
            this.intake.disableInnerIntakeMotor();
        } else if (hasTopBall && !hasBottomBall) {
            System.out.println(">>>INTAKE SLOW<<<");
            this.intake.innerIntakeLowSpeed();
        } else {
            System.out.println(">>>INTAKE<<<");
            this.intake.enableInnerIntakeMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.disableInnerIntakeMotor();
        this.intake.retractIntakeArmsSolenoid();
        this.intake.disableIntakeArmsMotor();
    }
}
