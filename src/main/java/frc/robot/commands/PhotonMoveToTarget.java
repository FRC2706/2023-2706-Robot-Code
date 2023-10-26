// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.*;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffTalonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonMoveToTarget extends CommandBase {
    double EXAMPLE_SIZE_HEIGHT = 25.808;
    double EXAMPLE_DISTANCE = 2.000;
    //height of april = 1 foot 3 and 1/4
  /** Creates a new ExampleCommand. */
  PhotonCamera camera1;
  PIDController xController;
  PIDController yController;
  PIDController yawController;
  public PhotonMoveToTarget() {
    camera1 = new PhotonCamera("OV9281");
    xController = new PIDController(0.7, 0, 0);
    yController = new PIDController(0.7, 0, 0);
    yawController = new PIDController(0.06, 0, 5);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera1.getLatestResult();
    if (result.hasTargets()){
      List<TargetCorner> corners = result.getBestTarget().getDetectedCorners();
      double height = (corners.get(0).y - corners.get(3).y + corners.get(1).y - corners.get(2).y)/2;
      double range = height/EXAMPLE_SIZE_HEIGHT*EXAMPLE_DISTANCE;
      double yaw = result.getBestTarget().getYaw();
      System.out.println(range);
      System.out.println(yaw);

    double xSpeed = xController.calculate(Math.cos(yaw)*range, 2);
    //double turnSpeed = yawController.calculate(yaw, 0);
    double yspeed = yController.calculate(Math.sin(yaw)*range, 2);
    
      SwerveSubsystem.getInstance().drive(xSpeed, yspeed, 0, false, false);
                              }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
