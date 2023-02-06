// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.ModuleAngleFromJoystick;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetGyroToNearest;
import frc.robot.commands.RotateAngleXY;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.translationCommand;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    configureTestJoystick();

    m_autoSelector = new AutoSelector();

    //use FRC Labview Dashboard
    String[] autoList = {"Test1", "Test2", "Test3", "To add more"};
    SmartDashboard.putStringArray("Auto List", autoList );
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

    SwerveSubsystem.getInstance().setDefaultCommand(new SwerveTeleop(driver));

    //Do not use driver's Right or Left Bumper, already used in separate file
    driver.start().onTrue(new ResetGyroToNearest());
    driver.back().onTrue(new ResetGyro());
    driver.b().onTrue(new InstantCommand(()-> SwerveSubsystem.getInstance().resetEncodersFromCanCoder()));
    
    driver.y().whileTrue(new RotateAngleXY(driver, 0));
    driver.a().whileTrue(new RotateAngleXY(driver, Math.PI));
    
    driver.x().whileTrue(new translationCommand(1, 1));
  }

  private void configureTestJoystick() {
    CommandXboxController testJoystick = new CommandXboxController(2);

    testJoystick.back().onTrue(new ResetGyro());
    testJoystick.start().onTrue(Commands.runOnce(
      () -> SwerveSubsystem.getInstance().resetOdometry(new Pose2d())));

    testJoystick.rightTrigger(0.5).whileTrue(new RotateAngleXY(testJoystick, 0));
    testJoystick.leftTrigger(0.5).whileTrue(new RotateAngleXY(testJoystick, Math.PI));

    SwerveModuleState forward = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    SwerveModuleState left = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
    SwerveModuleState diagonal = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

    double speed = 0.4;
    SwerveModuleState moveForward = new SwerveModuleState(speed, Rotation2d.fromDegrees(0));
    SwerveModuleState moveLeft = new SwerveModuleState(speed, Rotation2d.fromDegrees(90));
    SwerveModuleState moveDiagonal = new SwerveModuleState(speed, Rotation2d.fromDegrees(45));

    testJoystick.a().whileTrue(Commands.run(
      () -> SwerveSubsystem.getInstance().setModuleStates(
        new SwerveModuleState[]{forward, forward, forward, forward}, true, false),
        SwerveSubsystem.getInstance()));

    testJoystick.b().whileTrue(Commands.run(
      () -> SwerveSubsystem.getInstance().setModuleStates(
        new SwerveModuleState[]{left, left, left, left}, true, false),
        SwerveSubsystem.getInstance()));

    testJoystick.x().whileTrue(Commands.run(
      () -> SwerveSubsystem.getInstance().setModuleStates(
        new SwerveModuleState[]{diagonal, diagonal, diagonal, diagonal}, true, false),
        SwerveSubsystem.getInstance()));

    testJoystick.y().whileTrue(Commands.run(
      () -> SwerveSubsystem.getInstance().setModuleStates(
        new SwerveModuleState[]{moveForward, moveForward, moveForward, moveForward}, true, true),
        SwerveSubsystem.getInstance()));

    testJoystick.leftBumper().whileTrue(Commands.run(
      () -> SwerveSubsystem.getInstance().setModuleStates(
        new SwerveModuleState[]{moveLeft, moveLeft, moveLeft, moveLeft}, true, true),
        SwerveSubsystem.getInstance()));

    testJoystick.rightBumper().whileTrue(Commands.run(
      () -> SwerveSubsystem.getInstance().setModuleStates(
        new SwerveModuleState[]{moveDiagonal, moveDiagonal, moveDiagonal, moveDiagonal}, true, true),
        SwerveSubsystem.getInstance()));

    SwerveTeleop driveCommand = new SwerveTeleop(testJoystick);
    testJoystick.povDown().onTrue(driveCommand);
    testJoystick.povUp().onTrue(Commands.runOnce(() -> driveCommand.cancel()));


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
