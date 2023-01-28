// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.ModuleAngleFromJoystick;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SwerveTeleop;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class MiniSwerveContainer extends RobotContainer{

  AutoSelector m_autoSelector;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public MiniSwerveContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_autoSelector = new AutoSelector();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CommandXboxController driver = new CommandXboxController(0);

    SwerveSubsystem.getInstance().setDefaultCommand(new SwerveTeleop(driver, Config.Swerve.teleopSpeed, Config.Swerve.teleopAngularSpeed,5.0));

    driver.leftBumper().whileTrue(new SwerveTeleop(driver, Config.Swerve.teleopFastSpeed, Config.Swerve.teleopFastAngularSpeed,5.0));
    driver.rightBumper().whileTrue(new SwerveTeleop(driver, Config.Swerve.teleopSlowSpeed, Config.Swerve.teleopSlowAngularSpeed,20.0));
    driver.start().onTrue(new InstantCommand(()-> SwerveSubsystem.getInstance().updateModulesPID()));
    driver.back().onTrue(new ResetGyro());
    driver.y().onTrue(new InstantCommand(()-> SwerveSubsystem.getInstance().resetEncodersFromCanCoder()));


    SwerveModuleState state1 = new SwerveModuleState(0.3, Rotation2d.fromDegrees(0));
    SwerveModuleState state2 = new SwerveModuleState(0.3, Rotation2d.fromDegrees(90));
    SwerveModuleState state3 = new SwerveModuleState(0.5, Rotation2d.fromDegrees(0));
    SwerveModuleState state4 = new SwerveModuleState(-0.5, Rotation2d.fromDegrees(0));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    return m_autoSelector.getAutoCommand();
  }


}
