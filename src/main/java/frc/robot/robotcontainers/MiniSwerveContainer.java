// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.ControlRingLight;
import frc.robot.commands.ModuleAngleFromJoystick;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetGyroToNearest;
import frc.robot.commands.SwerveTeleop;
import frc.robot.config.Config;
import frc.robot.subsystems.RelaySubsystem;
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

    //use FRC Labview Dashboard
    String[] autoList = {"Test1", "Test2", "Test3", "To add more"};
    SmartDashboard.putStringArray("Auto List", autoList );

    //if (Config.robotId == 2) {
      RelaySubsystem.getInstance();
 // }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CommandXboxController driver = new CommandXboxController(0);

    SwerveSubsystem.getInstance().setDefaultCommand(new SwerveTeleop(driver));

    //Do not use Right or Left Bumper already used in separate file
    driver.start().onTrue(new ResetGyroToNearest());
    driver.back().onTrue(new ResetGyro());
    driver.y().onTrue(new InstantCommand(()-> SwerveSubsystem.getInstance().resetEncodersFromCanCoder()));

        //Rear small ring light
        Command controlRearSmallRinglight = new ControlRingLight(Config.RELAY_RINGLIGHT_REAR_SMALL);
        driver.a().onTrue(controlRearSmallRinglight);
        
        //Rear large ring light
        Command controlRearLargeRinglight = new ControlRingLight(Config.RELAY_RINGLIGHT_REAR_LARGE);
        driver.b().onTrue(controlRearLargeRinglight);
    
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
