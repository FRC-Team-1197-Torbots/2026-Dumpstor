// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {

  private Shooter shooter;
  private CommandXboxController controller;

  public RobotContainer() {
    shooter = new Shooter();
    controller = new CommandXboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(Commands.runOnce(() -> shooter.KickOn())).onFalse(Commands.runOnce(() -> shooter.KickOff()));
    controller.b().onTrue(Commands.runOnce(() -> shooter.TestDrum())).onFalse(Commands.runOnce(() -> shooter.StopDrum()));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
