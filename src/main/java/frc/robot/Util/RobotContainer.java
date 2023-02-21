// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Controls.ControlBoard;
import frc.robot.Drive.DriveWithJoystick;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.Intake;
import frc.robot.Shooter.LowShotCommand;
import frc.robot.Shooter.ShooterSubsystem;
// import frc.robot.Util.LEDs.LEDIdleCommand;
// import frc.robot.Util.LEDs.LEDSubsystem;

public class RobotContainer {

    private Drivetrain drivetrain;
    private Intake intake;
    private ShooterSubsystem shooter;
    // private LEDSubsystem ledSubsystem;
    private DriveWithJoystick driveWithJoystick;

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
        // ledSubsystem = new LEDSubsystem();
    }

    private void initControls() {
        ControlBoard.lowShotButton.whileTrue(new LowShotCommand(shooter));
        ControlBoard.intakeButton.onTrue(new InstantCommand(intake::enableInnerIntakeMotor))
        .onFalse(new InstantCommand(intake::disableInnerIntakeMotor));
    }

    public void init() {
        // this.ledSubsystem.setDefaultCommand(new LEDIdleCommand(this.ledSubsystem));
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

    public Intake getIntake() {
        return intake;
    }

    public void setTeleopDrive() {
        driveWithJoystick = new DriveWithJoystick(
            drivetrain, 
            ControlBoard.getDriverController()
        );
        drivetrain.setDefaultCommand(driveWithJoystick);
    }
}
