// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.robotcontainers;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class MiniSwerveContainer extends RobotContainer{

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public MiniSwerveContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  @Override
  public Command getAutonomousCommand() {

    
    Map<String, Command> eventMap = new HashMap<String, Command>();
    eventMap.put("intake", new InstantCommand(() -> System.out.println("intake")));
    eventMap.put("shoot", new InstantCommand(() -> System.out.println("shoot")));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      SwerveSubsystem.getInstance()::getPose, 
      SwerveSubsystem.getInstance()::resetOdometry, 
      frc.robot.config.Config.Swerve.kSwerveDriveKinematics, 
      new PIDConstants(0, 0, 0), 
      new PIDConstants(0, 0, 0), 
      SwerveSubsystem.getInstance()::setModuleStates,
      eventMap, 
      SwerveSubsystem.getInstance()
    );

    PathPlannerTrajectory basicPath = PathPlanner.loadPath("Basic path", new PathConstraints(4, 3));

    //return new InstantCommand();
    return autoBuilder.fullAuto(basicPath);
  }
}
