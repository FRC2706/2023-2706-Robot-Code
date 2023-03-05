// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.auto.AutoSelector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.auto.AutoRoutines;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetGyroToNearest;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RotateAngleXY;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.TranslationCommand;
import frc.robot.subsystems.RelaySubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class MiniSwerveContainer extends RobotContainer{





  Command armCommand;
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
    String[] autoList = {"Id0_null", 
                         "Id1_leave_top",
                         "Id2_leave_middle_around_",
                         "Id3_leave_middle_through_", 
                         "Id4_leave_bottom",
                         "Id5_leave_balance_top",
                         "Id6_leave_balance_middle_around_",
                         "Id7_leave_balance_middle_through_",
                         "Id8_leave_balance_bottom",
                         "Id9_place_pick_top",
                         "Id10_place_pick_middle_around_",
                         "Id11_place_pick_middle_through_",
                         "Id12_place_pick_bottom",
                         "Id13_place_pick_balance_top",
                         "Id14_place_pick_balance_middle",
                         "Id15_place_pick_balance_bottom",
                         "Id16_place_pick_place_balance_top",
                         "Id17_place_pick_place_balance_middle",
                         "Id18_place_pick_place_balance_bottom",
                         "Id19_place_pick_place_top",
                         "Id20_place_pick_place_middle",
                         "Id21_place_pick_place_bottom",
                         "Id22_place_pick_place_pick_place_top",
                         "Id23_place_pick_place_pick_place_middle",
                         "Id24_place_pick_place_pick_place_bottom",
                         "Id25_Practice1",
                         "Id26_Practice2",
                         "Id27_place_pick_place_pick_place_bottom_new",
                         "Id28_place_pick_bottom_charge_new",
                         "Id29_place_pick_top_charge_new",
                         "Id30_place_pick_place_pick_place_top2",
                        };
    SmartDashboard.putStringArray("Auto List", autoList );

    // Construct relays for MiniSwerve
    RelaySubsystem.getInstance();

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
    //driver.b().onTrue(new InstantCommand(()-> SwerveSubsystem.getInstance().resetEncodersFromCanCoder()));
    driver.b().onTrue(new ResetOdometry(new Pose2d()));

    driver.y().whileTrue(new RotateAngleXY(driver, 0));
    driver.a().whileTrue(new RotateAngleXY(driver, Math.PI));
    
    driver.x().whileTrue(new TranslationCommand(3.5, 0));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 
  @Override
  public Command getAutonomousCommand(){
    int autoId;
    String autoName = SmartDashboard.getString("Auto Selector", "null");
    switch(autoName)
    {
      case "Id0_null":
        autoId = 0;
        break;
      case "Id1_leave_top":
        autoId = 1;
        break;
      case "Id2_leave_middle_around_":
        autoId = 2;
        break;
      case "Id3_leave_middle_through":
        autoId = 3;
        break;
      case "Id4_leave_bottom":
        autoId = 4;
        break;
      case "Id5_leave_balance_top":
        autoId = 5;
        break;
      case "Id6_leave_balance_middle_around_":
        autoId = 6;
        break;
      case "Id7_leave_balance_middle_through_":
        autoId = 7;
        break;
      case "Id8_leave_balance_bottom":
        autoId = 8;
        break;
      case "Id9_place_pick_top":
        autoId = 9;
        break;

      case "Id10_place_pick_middle_around_":
        autoId = 10;
        break;
      case "Id11_place_pick_middle_through_":
        autoId = 11;
        break;
      case "Id12_place_pick_bottom":
        autoId = 12;
        break;
      case "Id13_place_pick_balance_top":
        autoId = 13;
        break;
      case "Id14_place_pick_balance_middle":
        autoId = 14;
        break;
      case "Id15_place_pick_balance_bottom":
        autoId = 15;
        break;
      case "Id16_place_pick_place_balance_top":
        autoId = 16;
        break;
      case "Id17_place_pick_place_balance_middle":
        autoId = 17;
        break;
      case "Id18_place_pick_place_balance_bottom":
        autoId = 18;
        break;
      case "Id19_place_pick_place_top":
        autoId = 19;
        break;
      case "Id20_place_pick_place_middle":
        autoId = 20;
        break;
      case "Id21_place_pick_place_bottom":
        autoId = 21;
        break;
      case "Id22_place_pick_place_pick_place_top":
        autoId = 22;
        break;
      case "Id23_place_pick_place_pick_place_middle":
        autoId = 23;
        break;
      case "Id24_place_pick_place_pick_place_bottom":
        autoId = 24;
        break;
      case "Id25_Practice1":
        autoId = 25;
        break;
      case "Id26_Practice2":
        autoId = 26;
        break;

      case "Id27_place_pick_place_pick_place_bottom_new": //2.5_bottom_new
        autoId = 27;
        break;
      
      case "Id28_place_pick_bottom_charge_new": //1_
        autoId = 28;
        break;
      
      case "Id29_place_pick_top_charge_new":
        autoId = 29;
        break;
      
      case "Id30_place_pick_place_pick_place_top2":
        autoId = 30;
        break;

      default:
        autoId = 0;
        break;
      
    }
    return routines.getAutonomousCommand(autoId);
  }
}
