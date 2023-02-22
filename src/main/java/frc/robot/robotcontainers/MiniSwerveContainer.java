// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.auto.AutoRoutines;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.AlignToTargetVision;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetGyroToNearest;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.RotateAngleXY;
import frc.robot.commands.RotateXYSupplier;
import frc.robot.commands.SwerveTeleop;
import frc.robot.commands.TranslationCommand;
import frc.robot.config.Config;
import frc.robot.subsystems.RelaySubsystem;
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
                         "leave_middle_around_",
                         "leave_middle_through_", 
                         "leave_bottom",
                         "leave_balance_top",
                         "leave_balance_middle_around_",
                         "leave_balance_middle_through_",
                         "leave_balance_bottom",
                         "place_pick_top",
                         "place_pick_middle_around_",
                         "place_pick_middle_through_",
                         "place_pick_bottom",
                         "place_pick_balance_top",
                         "place_pick_balance_middle",
                         "place_pick_balance_bottom",
                         "place_pick_place_balance_top",
                         "place_pick_place_balance_middle",
                         "place_pick_place_balance_bottom",
                         "place_pick_place_top",
                         "place_pick_place_middle",
                         "place_pick_place_bottom",
                         "place_pick_place_pick_place_top",
                         "place_pick_place_pick_place_middle",
                         "place_pick_place_pick_place_bottom",
                         "Practice1",
                         "Practice2"
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
    
    //driver.b().whileTrue(new RotateXYSupplier(driver, ()-> visionYaw.get()));
    //driver.b().whileTrue(new ResetOdometry(new Pose2d()));
    driver.b().onTrue(new ResetOdometry(new Pose2d()));

    driver.y().whileTrue(new RotateAngleXY(driver, 0));
    driver.a().whileTrue(new RotateAngleXY(driver, Math.PI));
    
    driver.x().whileTrue(new TranslationCommand(1, 1));
    driver.leftTrigger().whileTrue(new RotateXYSupplier(driver,
      NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99)
    ));

    driver.rightTrigger().whileTrue(Commands.sequence(
      new AlignToTargetVision(driver,
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99),
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("DistanceToTarget").subscribe(-99),
        2.0,
        0.3),
      new AlignToTargetVision(driver,
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("YawToTarget").subscribe(-99),
        NetworkTableInstance.getDefault().getTable("pipelineTape21").getDoubleTopic("DistanceToTarget").subscribe(-99),
        1.5,
        0.03)
    ));

    RelaySubsystem.getInstance().setRelay(Config.RELAY_RINGLIGHT_REAR_LARGE, true);
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
      case "null":
        autoId = 0;
        break;
      case "leave_top":
        autoId = 1;
        break;
      case "leave_middle_around_":
        autoId = 2;
        break;
      case "leave_middle_through":
        autoId = 3;
        break;
      case "leave_bottom":
        autoId = 4;
        break;
      case "leave_balance_top":
        autoId = 5;
        break;
      case "leave_balance_middle_around_":
        autoId = 6;
        break;
      case "leave_balance_middle_through_":
        autoId = 7;
        break;
      case "leave_balance_bottom":
        autoId = 8;
        break;
      case "place_pick_top":
        autoId = 9;
        break;

      case "place_pick_middle_around_":
        autoId = 10;
        break;
      case "place_pick_middle_through_":
        autoId = 11;
        break;
      case "place_pick_bottom":
        autoId = 12;
        break;
      case "place_pick_balance_top":
        autoId = 13;
        break;
      case "place_pick_balance_middle":
        autoId = 14;
        break;
      case "place_pick_balance_bottom":
        autoId = 15;
        break;
      case "place_pick_place_balance_top":
        autoId = 16;
        break;
      case "place_pick_place_balance_middle":
        autoId = 17;
        break;
      case "place_pick_place_balance_bottom":
        autoId = 18;
        break;
      case "place_pick_place_top":
        autoId = 19;
        break;
      case "place_pick_place_middle":
        autoId = 20;
        break;
      case "place_pick_place_bottom":
        autoId = 21;
        break;
      case "place_pick_place_pick_place_top":
        autoId = 22;
        break;
      case "place_pick_place_pick_place_middle":
        autoId = 23;
        break;
      case "place_pick_place_pick_place_bottom":
        autoId = 24;
        break;
      case "Practice1":
        autoId = 25;
        break;
      case "Practice2":
        autoId = 26;
        break;
      default:
        autoId = 0;
        break;
      
    }
    return routines.getAutonomousCommand(autoId);
  }
}
