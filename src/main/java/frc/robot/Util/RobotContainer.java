
package frc.robot.Util;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Climber.ClimberMotorsSubsystem;
import frc.robot.Climber.SensorWinchRetract;
import frc.robot.Climber.WinchExtend;
import frc.robot.Climber.WinchRetract;
import frc.robot.Controls.ControlBoard;
import frc.robot.Drive.DriveWithJoystick;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.LowShotCommand;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Util.LEDs.LEDIdleCommand;
import frc.robot.Util.LEDs.LEDSubsystem;

public class RobotContainer {

    private Drivetrain drivetrain;
    private Intake intake;
    private ShooterSubsystem shooter;
    private ClimberMotorsSubsystem winch;
    private LEDSubsystem ledSubsystem;
    private DriveWithJoystick driveWithJoystick;

    TrajectorySelector trajectorySelector = new TrajectorySelector(
            Filesystem.getDeployDirectory().toPath().resolve("paths/"), true);
    public Field2d robotFieldWidget = new Field2d();

    public RobotContainer() {
        ControlBoard.init();
        initSubsystems();
        initControls();
        LimeLight.init();
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
    }

    private void initSubsystems() {
        drivetrain = new Drivetrain();
        intake = new Intake();
        shooter = new ShooterSubsystem();
        winch = new ClimberMotorsSubsystem();
        ledSubsystem = new LEDSubsystem();
    }

    private void initControls() {
        ControlBoard.lowShotButton.whileActiveOnce(new LowShotCommand(shooter));
        ControlBoard.intakeButton.whenPressed(new InstantCommand(intake::enableInnerIntakeMotor))
        .whenReleased(new InstantCommand(intake::disableInnerIntakeMotor));
        ControlBoard.extendArms.whenPressed(new WinchExtend(winch, Constants.winchMaxLimit));
        ControlBoard.retractArms.whenPressed(new SensorWinchRetract(winch));
    }

    public void init() {
        this.ledSubsystem.setDefaultCommand(new LEDIdleCommand(this.ledSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public void logShot() {
        DataLogManager.log("LeftRPM: " + this.getShooterSubsystem().getLeftRPM() + " RightRPM: "
                + this.getShooterSubsystem().getRightRPM() + " RPMSetpoint: " + this.getShooterSubsystem().getSetpoint()
                + " AtSetpoint: " + this.getShooterSubsystem().atTargetRPM() + " X LimeLight: " + LimeLight.getTX()
                + " Y LimeLight: " + LimeLight.getTY() + " EventName: " + DriverStation.getEventName()
                + " MatchNumber: " + DriverStation.getMatchNumber() + " MatchTime: " + DriverStation.getMatchTime());
    }

    public Drivetrain getDriveTrain() {
        return drivetrain;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooter;
    }

    public TrajectorySelector getTrajectorySelector() {
        return trajectorySelector;
    }

    public Intake getIntake() {
        return intake;
    }

    public ClimberMotorsSubsystem getClimberMotorsSubsystem() {
        return winch;
    }

    public void setTeleopDrive() {
        driveWithJoystick = new DriveWithJoystick(
            drivetrain, 
            ControlBoard.getDriverController()
        );
        drivetrain.setDefaultCommand(driveWithJoystick);
    }
}
