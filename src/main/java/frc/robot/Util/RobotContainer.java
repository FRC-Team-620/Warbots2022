// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Climber.ClimberMotorsSubsystem;
import frc.robot.Climber.ClimberSubsystem;
import frc.robot.Climber.ExtendArmsAndStow;
import frc.robot.Climber.RaiseAndGrab;
import frc.robot.Climber.RaisePistons;
import frc.robot.Climber.ToggleHooks;
import frc.robot.Climber.WinchHold;
import frc.robot.Controls.ControlBoard;
import frc.robot.Drive.DriveWithJoystick;
import frc.robot.Drive.Drivetrain;
import frc.robot.Loader.Intake;
import frc.robot.Loader.IntakeBall;
import frc.robot.Loader.OuttakeBall;
import frc.robot.Shooter.ActivateFiringPins;
import frc.robot.Shooter.FiringPins;
import frc.robot.Shooter.LazySusanSubsystem;
import frc.robot.Shooter.LimelightSpinUp;
import frc.robot.Shooter.LowShotCommand;
import frc.robot.Shooter.ManualAimingPID;
import frc.robot.Shooter.ShooterSubsystem;
import frc.robot.Shooter.TankDriveAiming;
import frc.robot.Shooter.TurretAimingPID;
import frc.robot.Shooter.ZeroTurnTable;

/** Add your docs here. */
public class RobotContainer {

    // initialize subsystems
    private Drivetrain drivetrain;
    private Intake intake;
    private ShooterSubsystem shooter;
    private FiringPins firingPins;
    private LazySusanSubsystem turret;
    private ClimberSubsystem climberHooks;
    private ClimberMotorsSubsystem winch;

    private DriveWithJoystick driveWithJoystick;

    // TODO: *sighs emoji*
    TrajectorySelector trajectorySelector = new TrajectorySelector(
            Filesystem.getDeployDirectory().toPath().resolve("paths/"), true);
    public Field2d robotFieldWidget = new Field2d(); // TODO: include Robot odometry

    public RobotContainer() {
		ControlBoard.init();
        initSubsystems();
        initControls();
        LimeLight.init();
    }

    private void initSubsystems() {
        drivetrain = new Drivetrain();
        intake = new Intake();
        shooter = new ShooterSubsystem();
        firingPins = new FiringPins();
        turret = new LazySusanSubsystem(drivetrain::getPose);
        climberHooks = new ClimberSubsystem();
        winch = new ClimberMotorsSubsystem();
        SmartDashboard.putData(new ZeroTurnTable(turret));
    }

    private void initControls() {
        // operator
        ControlBoard.raiseArmsButton.whenPressed(
                new RaisePistons(climberHooks));

        ControlBoard.extendArmsButton.whenPressed(
                new ParallelCommandGroup(
                        new ExtendArmsAndStow(winch, climberHooks, intake)
                        //new DirectTurret(turret, shooter, Constants.stowedPosition)
                        ));

        ControlBoard.climbSequenceButton.whenPressed(
                new RaiseAndGrab(winch, climberHooks));

        // controls.tankDriveAimButton.whileActiveOnce(
        //     new TankDriveAutoAimAndSpinUp(getShooterSubsystem(), getDriveTrain(), 
        //         false, controls.getOperatorController()));

        ControlBoard.lowerHooksButton.whenPressed(
                new ToggleHooks(climberHooks));

        ControlBoard.winchHoldButton.whenPressed(
                new WinchHold(winch, winch.getWinchPosition(), Constants.holdTime));

        // TODO: here, now make a unified aiming/flywheel spinup command that we can use
        // for both auto and tele


        ControlBoard.aimTurretTrigger.whileActiveOnce(
            new ParallelCommandGroup(   
                new LimelightSpinUp(this.getShooterSubsystem()),
                //new TurretAiming(this.getLazySusanSubsystem())
                new TurretAimingPID(this.getLazySusanSubsystem(), robotFieldWidget, drivetrain::getPose)
            ));

        ControlBoard.tankDriveAimButton.whileActiveOnce(
            new ParallelCommandGroup(
                new LimelightSpinUp(this.getShooterSubsystem()),
                new TankDriveAiming(this.getDriveTrain())
            ));
            // new AutoAimingAndSpinningUp(getShooterSubsystem(), getLazySusanSubsystem(), 
            //     false, controls.getOperatorController()));

        ControlBoard.toggleGyroButton.whenPressed(
            new InstantCommand(() -> this.getLazySusanSubsystem().setIsGyroLocking(
                !this.getLazySusanSubsystem().getIsGyroLocking()
            ))
        );

        ControlBoard.fireTurretTrigger.whenActive(
        new ActivateFiringPins(getFiringPins()));

        //driver
        ControlBoard.lowShotButton.whileActiveOnce(new LowShotCommand(shooter));

        ControlBoard.intakeButton.whileActiveOnce(new IntakeBall(intake));

        ControlBoard.outakeButton.whileActiveOnce(new OuttakeBall(intake));

        // controls.aimTurretTrigger.whenActive(
        // new AimTurretCommand();
        // );
        // controls.fireTurretTrigger.whenPressed(
        // new FireTriggerCommand();
        // );

    }

    public void init() {
        // TODO: :)))))))))
        // will fix this later

        // only valid for now so this is still functional and builds
        // driveWithJoystick = new DriveWithJoystick(drivetrain, controls.getDriverController(),
        //         controls.getOperatorController());
        // drivetrain.setDefaultCommand(driveWithJoystick);

        turret.setDefaultCommand(new ManualAimingPID(turret, ControlBoard.getOperatorController()));
        //TODO: setup turret
        // turret.setDefaultCommand(new TurretAimingPID(turret));
        // shooter.setDefaultCommand(new LimelightSpinUp(shooter));

        //shooterCommand = new ShooterCommand(shooter, turret, controls.getOperatorController(),
                //controls.getDriverController());
        //shooter.setDefaultCommand(shooterCommand);
        // shooterSubsystem.setDefaultCommand( new
        // PIDShooterCommand(shooterSubsystem));//Show off pid shooter cmd Only works in
        // sim rn

        // climberCommand = new ClimberCommand(climberSubsystem);
        // climberMotorsSubsystem.setDefaultCommand(new
        // ClimberManual(climberMotorsSubsystem, operator));
        new ToggleHooks(climberHooks).schedule();

        SmartDashboard.putData(robotFieldWidget);
        SmartDashboard.putData(trajectorySelector);
        robotFieldWidget.getObject("Turret").setPose(new Pose2d());
        trajectorySelector.linkField(robotFieldWidget);

        // trajectorySelector.setDefaultOption("No Trajectory", new Trajectory());
        // //Uncomment this to default to no trajectory vs the first file found or null.

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public ClimberSubsystem getClimberSubsystem() {
        return climberHooks;
    }

    public LazySusanSubsystem getLazySusanSubsystem() {
        return turret;
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

    // public LoaderCommand getLoaderCommand() {
    //     return loaderCommand;
    // }

    public Intake getIntake() {
        return intake;
    }

    public FiringPins getFiringPins() {
        return firingPins;
    }

    public XboxController getOperatorController() {
        return ControlBoard.getOperatorController();
    }

    public ClimberMotorsSubsystem getClimberMotorsSubsystem() {
        return winch;
    }

    public void setTeleopDrive() {
        driveWithJoystick = new DriveWithJoystick(drivetrain, ControlBoard.getDriverController(),
            ControlBoard.getOperatorController());
        drivetrain.setDefaultCommand(driveWithJoystick);
    }

    //public Command getAutonomousCommand(Trajectory traj) {
    //    // Create a voltage constraint to ensure we don't accelerate too fast
    //    System.out.println("Auto Path Ran");
    //    // var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    //   //         new SimpleMotorFeedforward(
    //    //                 Constants.ksVolts,
    //   //                 Constants.kvVoltSecondsPerMeter,
    //    //                 Constants.kaVoltSecondsSquaredPerMeter),
    //    //         Constants.kDriveKinematics,
    //    //         5.02);

    //    // // Create config for trajectory
    //    // TrajectoryConfig config = new TrajectoryConfig(
    //    //         Constants.kMaxSpeedMetersPerSecond,
    //    //         Constants.kMaxAccelerationMetersPerSecondSquared)
    //    //         // Add kinematics to ensure max speed is actually obeyed
    //    //         .setKinematics(Constants.kDriveKinematics)
    //    //         // Apply the voltage constraint
    //    //         .addConstraint(autoVoltageConstraint);

    //    // // An example trajectory to follow. All units in meters.
    //    // Trajectory exampleTrajectory =
    //    // TrajectoryGenerator.generateTrajectory(
    //    // // Start at the origin facing the +X direction
    //    // new Pose2d(0, 0, new Rotation2d(0)),
    //    // // Pass through these two interior waypoints, making an 's' curve path
    //    // List.of(new Translation2d(0.5, 0)),//, new Translation2d(2, -1)//1
    //    // // End 3 meters straight ahead of where we started, facing forward
    //    // new Pose2d(1, 0, new Rotation2d(0)),
    //    // // Pass config
    //    // config);
    //    // Trajectory jsonTrajectory = trajectorySelector.getSelected();
    //    Trajectory jsonTrajectory = traj;

    //    RamseteCommand ramseteCommand = new RamseteCommand(
    //            jsonTrajectory,
    //            drivetrain::getPose,
    //            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    //            new SimpleMotorFeedforward(
    //                    Constants.ksVolts,
    //                    Constants.kvVoltSecondsPerMeter,
    //                    Constants.kaVoltSecondsSquaredPerMeter),
    //           Constants.kDriveKinematics,
    //            drivetrain::getWheelSpeeds,
    //            new PIDController(Constants.kPDriveVel, 0, 0),
    //            new PIDController(Constants.kPDriveVel, 0, 0),
    //            // RamseteCommand passes volts to the callback
    //            drivetrain::tankDriveVolts,
    //           drivetrain);

    //    // Reset odometry to the starting pose of the trajectory.
    //    drivetrain.resetOdometry(jsonTrajectory.getInitialPose());

    //    // Run path following command, then stop at the end.
    //    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    //}

}
