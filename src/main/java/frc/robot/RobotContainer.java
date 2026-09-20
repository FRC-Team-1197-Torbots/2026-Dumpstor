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

    private final CommandGamepad joystick = new CommandGamepad(0);

    // Uncommented the drum subsystem
    private final Drum drum = new Drum();
    private final Intake intake = new Intake();
    private final Kicker kicker = new Kicker();
    private final Floor floor = new Floor();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final ShootCommand shoot = new ShootCommand(drum, floor, kicker, intake, ShooterConstants.IdleShooterSpeed);

    // private final org.wpilib.smartdashboard.SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register Named Commands for PathPlanner
        NamedCommands.registerCommand("Shoot", shoot);
        NamedCommands.registerCommand("Toggle Intake", intake.toggleDeployCommand());

        // Setup PathPlanner before generating the chooser
        drivetrain.setupPathPlanner();
        
        configureBindings();

        // Build an auto chooser from the autos configured in PathPlanner GUI
        // autoChooser = com.pathplanner.lib.auto.AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        drum.setDefaultCommand(Commands.runOnce(()-> drum.setVelocityRPS(ShooterConstants.IdleShooterSpeed), drum));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        // Reset the field-centric heading on left bumper press.
        joystick.button(3).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // ==========================================
        // DRUM TUNING BINDINGS
        // ==========================================
        
        // Hold A: Slowly ramp voltage by 0.1V/sec to find kS (releases to 0V)
        // joystick.button(0).whileTrue(kicker.feedShooterCommand());
        // joystick.button(1).whileTrue(kicker.unjamCommand());

        // joystick.button(1).whileTrue(Commands.runOnce(() -> drum.setVelocityRPS(ShooterConstants.IdleShooterSpeed), drum));
        // joystick.button(2).whileTrue(floor.unjamCommand());

        joystick.button(10).whileTrue(intake.intakeGamePieceCommand());
        joystick.button(9).whileTrue(intake.ejectGamePieceCommand()); 
        
        joystick.button(0).whileTrue(intake.toggleDeployCommand());

        joystick.axisGreaterThan(5, 0.1f).whileTrue(shoot);      
    }


    public Command getAutonomousCommand() {
        // Returns the command selected in the SmartDashboard dropdown 
        // generated by PathPlanner's AutoBuilder.
        return null; // autoChooser.getSelected();
    }
}