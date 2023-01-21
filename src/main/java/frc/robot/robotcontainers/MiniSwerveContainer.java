// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

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
    Joystick driver = new Joystick(0);

    SwerveSubsystem.getInstance().setDefaultCommand(new SwerveTeleop(driver, Config.Swerve.teleopSpeed, Config.Swerve.teleopAngularSpeed,5.0));
    
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whenHeld(
      new SwerveTeleop(driver, Config.Swerve.teleopFastSpeed, Config.Swerve.teleopFastAngularSpeed,5.0));

    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whenHeld(
      new SwerveTeleop(driver, Config.Swerve.teleopSlowSpeed, Config.Swerve.teleopSlowAngularSpeed,20.0));

    new JoystickButton(driver, XboxController.Button.kStart.value).whenPressed(
      new InstantCommand(()-> SwerveSubsystem.getInstance().updateModulesPID())
    );

    new JoystickButton(driver, XboxController.Button.kBack.value).whenPressed(new ResetGyro());

    new JoystickButton(driver, XboxController.Button.kY.value).whenPressed(
      new InstantCommand(()-> SwerveSubsystem.getInstance().resetEncodersFromCanCoder())
    );
    
    SwerveModuleState state1 = new SwerveModuleState(0.3, Rotation2d.fromDegrees(0));
    SwerveModuleState state2 = new SwerveModuleState(0.3, Rotation2d.fromDegrees(90));
    SwerveModuleState state3 = new SwerveModuleState(0.5, Rotation2d.fromDegrees(0));
    SwerveModuleState state4 = new SwerveModuleState(-0.5, Rotation2d.fromDegrees(0));


    // Command angleSetPoint1 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state1, state1, state1, state1}, true), SwerveSubsystem.getInstance());
    // new JoystickButton(driver, XboxController.Button.kB.value).whenHeld(angleSetPoint1).whenReleased(new InstantCommand(SwerveSubsystem.getInstance() :: stopMotors, SwerveSubsystem.getInstance()));

    // Command angleSetPoint2 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state2, state1, state1, state1}, true), SwerveSubsystem.getInstance());
    // new JoystickButton(driver, XboxController.Button.kX.value).whenHeld(angleSetPoint2).whenReleased(new InstantCommand(SwerveSubsystem.getInstance() :: stopMotors, SwerveSubsystem.getInstance()));

    // Command angleSetPoint3 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state2, state2, state2, state2}, true), SwerveSubsystem.getInstance());
    // new JoystickButton(driver, XboxController.Button.kA.value).whenHeld(angleSetPoint3).whenReleased(new InstantCommand(SwerveSubsystem.getInstance() :: stopMotors, SwerveSubsystem.getInstance()));

    // Command angleSetPoint4 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state3, state3, state3, state3}, true), SwerveSubsystem.getInstance());
    // new JoystickButton(driver, XboxController.Button.kRightBumper.value).whenHeld(angleSetPoint4).whenReleased(new InstantCommand(SwerveSubsystem.getInstance() :: stopMotors, SwerveSubsystem.getInstance()));

    // Command angleSetPoint5 = new RunCommand(() -> SwerveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state4, state4, state4, state4}, true), SwerveSubsystem.getInstance());
    // new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whenHeld(angleSetPoint5).whenReleased(new InstantCommand(SwerveSubsystem.getInstance() :: stopMotors, SwerveSubsystem.getInstance()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {
    Map<String, Command> eventMap = new HashMap<String, Command>();
    eventMap.put("intake", new InstantCommand(() -> System.out.println("intake")));
    eventMap.put ("shoot", new InstantCommand (() -> System.out.println("shoot")));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      SwerveSubsystem.getInstance():: getPose,
      SwerveSubsystem.getInstance():: resetOdometry,
      Config.Swerve.kSwerveDriveKinematics,
      new PIDConstants (0,0,0),
      new PIDConstants (0,0,0),
      SwerveSubsystem.getInstance() :: setModuleStatesAuto,
      eventMap,
      false,
      SwerveSubsystem.getInstance()
      );

      PathPlannerTrajectory path = PathPlanner.loadPath("New New Path",1 ,1);
    return autoBuilder.fullAuto(path);

  }


}
