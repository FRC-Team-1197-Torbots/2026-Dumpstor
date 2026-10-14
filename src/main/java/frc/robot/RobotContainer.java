// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.tunable.Tunables;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TorbotsSubsystems.Drum;
import frc.robot.subsystems.TorbotsSubsystems.Floor;
import frc.robot.subsystems.TorbotsSubsystems.Intake;
import frc.robot.subsystems.TorbotsSubsystems.Kicker;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Gamepad controllers (Port 0: Driver, Port 1: Operator)
    private final CommandGamepad joystick = new CommandGamepad(0);
    private final CommandGamepad operatorJoystick = new CommandGamepad(1);

    // Robot Subsystems
    private final Drum drum = new Drum();       // Main 4-motor flywheel shooter
    private final Intake intake = new Intake(); // Dual-motor deploy rack & dual-motor intake rollers
    private final Kicker kicker = new Kicker(); // Accelerator rollers feeding into drum flywheels
    private final Floor floor = new Floor();   // Floor hopper belts/rollers

    // CTRE Swerve Drivetrain instance generated via Tuner X
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Vision subsystem (PhotonVision multi-camera AprilTag pose estimation)
    // private final frc.robot.subsystems.Vision vision = new frc.robot.subsystems.Vision(drivetrain);

    // Teleoperated shooting commands:
    // Continuous execution (held while trigger is active). agitateIntake is set to false
    // so the human operator has manual control over intake agitation via operatorJoystick.
    private final ShootCommand shoot = new ShootCommand(drum, kicker, intake, ShooterConstants.LowShooterSpeed);
    private final ShootCommand maxShoot = new ShootCommand(drum, kicker, intake, ShooterConstants.MaxShooterSpeed);

    // Auto Chooser for PathPlanner autonomous routines
    private final org.wpilib.tunable.Selectable<Command> autoChooser;

    public RobotContainer() {
        // =========================================================================
        // PATHPLANNER NAMED COMMANDS
        // =========================================================================
        // Registering named commands allows PathPlanner autonomous routines (.auto/.path)
        // to call Java commands by name at event markers or sequential steps.

        // "Shoot": Primary autonomous shooting command. Runs up to speed, feeds for
        // kAutoShootDurationSeconds (2.5s) while agitating the intake, then ends cleanly.
        NamedCommands.registerCommand("Shoot", createAutoShootCommand());
        NamedCommands.registerCommand("Shoot With Agitation", createAutoShootCommand(true));
        NamedCommands.registerCommand("Shoot Without Agitation", createAutoShootCommand(false));

        // Standalone intake agitation command for autonomous (1.5s non-blocking oscillation)
        NamedCommands.registerCommand("Agitate Intake", intake.agitateCommand());

        // Intake deploy and roller commands
        NamedCommands.registerCommand("Toggle Intake", intake.toggleDeployCommand());
        NamedCommands.registerCommand("Deploy Intake", intake.deployIntakeCommand());
        NamedCommands.registerCommand("Retract Intake", intake.retractIntakeCommand());
        NamedCommands.registerCommand("Intake Game Piece", intake.intakeGamePieceCommand());

        // Configure PathPlanner auto-builder with drivetrain feedback & odometry
        drivetrain.setupPathPlanner();
        
        // Configure controller button and trigger bindings
        configureBindings();

        // Build the auto chooser from the autos configured in deploy/pathplanner/autos/
        autoChooser = com.pathplanner.lib.auto.AutoBuilder.buildAutoChooser("Simple Auto");
        Tunables.publish("Auto/AutoSelection", autoChooser);
    }

    /**
     * Factory method creating a fresh, timed ShootCommand for autonomous.
     * Unlike teleoperated shooting, this command is guaranteed to end after
     * kAutoShootDurationSeconds so sequential autonomous steps can execute.
     */
    public Command createAutoShootCommand() {
        return createAutoShootCommand(true);
    }

    public Command createAutoShootCommand(boolean agitate) {
        return createAutoShootCommand(ShooterConstants.kAutoShootDurationSeconds, agitate);
    }

    public Command createAutoShootCommand(double durationSeconds, boolean agitate) {
        return new ShootCommand(
            drum,
            kicker,
            intake,
            ShooterConstants.LowShooterSpeed,
            durationSeconds,
            agitate
        );
    }

    /**
     * Applies a quadratic response curve to joystick axis inputs while preserving sign.
     * Small stick inputs give precise, smooth micro-adjustments for lining up with
     * targets, while full stick deflection still yields 100% of maximum speed.
     *
     * Formula: sign(input) * (input)^2
     */
    private static double applyStickCurve(double input) {
        return Math.copySign(input * input, input);
    }

    /**
     * Configure teleoperated gamepad bindings for driver and operator.
     * Driver controls robot navigation, shooting triggers, and basic intaking.
     * Operator controls secondary mechanisms (unjamming, squaring, agitating).
     */
    private void configureBindings() {
        // =========================================================================
        // DRIVETRAIN CONTROLS (Field-Centric Swerve)
        // =========================================================================
        // WPILib convention: +X is forward (towards red alliance wall), +Y is left.
        // Joystick conventions: pushing forward on stick gives negative Y.
        // applyStickCurve provides fine precision near center stick and full speed at max throw.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(applyStickCurve(-joystick.getLeftY()) * MaxSpeed)         // Drive forward/backward
                    .withVelocityY(applyStickCurve(-joystick.getLeftX()) * MaxSpeed)         // Strafe left/right
                    .withRotationalRate(applyStickCurve(-joystick.getRightX()) * MaxAngularRate) // Rotate
            )
        );

        // Drum Flywheel Default Command:
        // Keeps flywheels idling at IdleShooterSpeed when not actively shooting.
        // Using drum.run() ensures the command never prematurely ends.
        drum.setDefaultCommand(drum.run(() -> drum.setVelocityRPS(ShooterConstants.IdleShooterSpeed)));

        // Brake swerve modules in neutral while disabled
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Zero intake deploy encoders when autonomous starts (robot starts within frame perimeter)
        RobotModeTriggers.autonomous().onTrue(
            Commands.runOnce(() -> intake.zeroDeployEncoder()).ignoringDisable(true)
        );

        // Driver Button 3 (X / Left Bumper): Re-seed field-centric gyro heading
        joystick.button(3).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        // Driver Button 2: Manual override to zero intake deploy encoder
        joystick.button(2).onTrue(Commands.runOnce(() -> intake.zeroDeployEncoder()).ignoringDisable(true));

        // Driver Button 10: Hold to run intake rollers forward (intake game piece)
        joystick.button(10).whileTrue(intake.intakeGamePieceCommand());

        // Driver Button 9: Hold to run intake rollers in reverse (eject game piece)
        joystick.button(9).whileTrue(intake.ejectGamePieceCommand()); 
        
        // Driver Button 0 (A): Toggle deploy between stowed (0.0 rots) and extended (12.75 rots)
        joystick.button(0).whileTrue(intake.toggleDeployCommand());

        // =========================================================================
        // DRIVER SHOOTING TRIGGERS
        // =========================================================================
        var rightTrigger = joystick.axisGreaterThan(org.wpilib.driverstation.Gamepad.Axis.RIGHT_TRIGGER, 0.1f);
        var leftTrigger = joystick.axisGreaterThan(org.wpilib.driverstation.Gamepad.Axis.LEFT_TRIGGER, 0.1f);

        // Right Trigger only: Low speed shooting (4000 RPM) while locking drivetrain in brake
        rightTrigger.and(leftTrigger.negate()).whileTrue(shoot.alongWith(drivetrain.applyRequest(() -> brake)));

        // Right Trigger + Left Trigger: Max speed shooting (4800 RPM) while locking drivetrain in brake
        rightTrigger.and(leftTrigger).whileTrue(maxShoot.alongWith(drivetrain.applyRequest(() -> brake)));

        // =========================================================================
        // OPERATOR CONTROLS
        // =========================================================================
        // Operator Button 0 (A): Square intake against the hard stop to correct any physical skew
        operatorJoystick.button(0).onTrue(intake.squareIntakeCommand());

        // Operator Button 10 (Left Bumper): Manually agitates intake halfway while held (useful during shooting)
        operatorJoystick.button(10).whileTrue(intake.operatorAgitateCommand());

        // Operator Button 1 (B): Retract intake to 0.0 (in case of jam against bumper)
        operatorJoystick.button(1).onTrue(intake.retractIntakeCommand());
    }

    /**
     * Called by Robot.java during autonomousInit().
     * Sets drum to idle speed and returns the autonomous command chosen on the dashboard.
     */
    public Command getAutonomousCommand() {
        drum.setVelocityRPS(ShooterConstants.IdleShooterSpeed);
        return autoChooser.getSelected();
    }
}