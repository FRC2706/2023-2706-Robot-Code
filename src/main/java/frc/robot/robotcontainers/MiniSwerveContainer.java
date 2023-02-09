// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.auto.AutoRoutines;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetGyroToNearest;
import frc.robot.commands.RotateAngleXY;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.translationCommand;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class MiniSwerveContainer extends RobotContainer{

  AutoSelector m_autoSelector;


  AutoRoutines routines;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public MiniSwerveContainer() {
    LiveWindow.enableAllTelemetry();
    // Configure the button bindings
    configureButtonBindings();

    m_autoSelector = new AutoSelector();
    routines = new AutoRoutines();

    //use FRC Labview Dashboard
    String[] autoList = {"null", 
                         "leave_top",
                         "leave_middle_through", 
                         "leave_bottom",
                         "leave_balance_bottom",
                         "place_pick_top",
                         "place_pick_middle_around_",
                         "place_pick_middle_through_",
                         "place_pick_bottom",
                         "place_pick_balance_top",
                         "place_pick_balance_middle",
                         "place_pick_balance_bottom",
                        };
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

    SwerveSubsystem.getInstance().setDefaultCommand(new SwerveTeleop(driver));

    //Do not use Right or Left Bumper already used in separate file
    driver.start().onTrue(new ResetGyroToNearest());
    driver.back().onTrue(new ResetGyro());
    driver.b().onTrue(new InstantCommand(()-> SwerveSubsystem.getInstance().resetEncodersFromCanCoder()));
    
    driver.y().whileTrue(new RotateAngleXY(driver, 0));
    driver.a().whileTrue(new RotateAngleXY(driver, Math.PI));
    
    driver.x().whileTrue(new translationCommand(1, 1));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand(){
    int autoId;
    String autoName = SmartDashboard.getString("Auto Selector", "Now!");
    switch(autoName)
    {
      case "null":
        autoId = 0;
        break;
      case "leave_top":
        autoId = 1;
        break;
      case "leave_middle_through":
        autoId = 2;
        break;
      case "leave_bottom":
        autoId = 3;
        break;
      case "leave_balance_bottom":
        autoId = 4;
        break;
      case "place_pick_top":
        autoId = 5;
        break;
      case "place_pick_middle_around_":
        autoId = 6;
        break;
      case "place_pick_middle_through_":
        autoId = 7;
        break;
      case "place_pick_bottom":
        autoId = 8;
        break;
      case "place_pick_balance_top":
        autoId = 9;
        break;
      case "place_pick_balance_middle":
        autoId = 10;
        break;
      case "place_pick_balance_bottom":
        autoId = 11;
        break;
      default:
        autoId=0;
        break;
      
    }
    return routines.getAutonomousCommand(autoId);
  }
}
