// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.BeetleLimelightAlign;
import frc.robot.config.Config;
import frc.robot.subsystems.DiffTalonSubsystem;
import frc.robot.subsystems.RelaySubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class BeetleContainer extends RobotContainer {

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public BeetleContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CommandXboxController driver = new CommandXboxController(0);
    CommandXboxController operator = new CommandXboxController(1);

    DiffTalonSubsystem.getInstance().setDefaultCommand(
        new ArcadeDrive(driver, 1, 2));// XboxController.Axis.kLeftY.value, XboxController.Axis.kRightX.value));

    driver.a().whileTrue(new BeetleLimelightAlign());
    
    // Construct RelaySubsystem to the Relays can be controlled by NetworkTables
    RelaySubsystem.getInstance();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    return new InstantCommand(); 
  }
}
