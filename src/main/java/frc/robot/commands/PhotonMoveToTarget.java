// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffTalonSubsystem;

public class PhotonMoveToTarget extends CommandBase {
    double CAMERA_HEIGHT_METERS = 0.255;
    double TARGET_HEIGHT_METERS = 0.85;
    double CAMERA_PITCH_RADIANS = Units.degreesToRadians(14);
  /** Creates a new ExampleCommand. */
  PhotonCamera camera1;
  PIDController rangeController;
  PIDController yawController;
  public PhotonMoveToTarget() {
    camera1 = new PhotonCamera("OV9281");
    rangeController = new PIDController(-.7, 0, 0);
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
        double range =
                        PhotonUtils.calculateDistanceToTargetMeters(
                                CAMERA_HEIGHT_METERS,
                                TARGET_HEIGHT_METERS,
                                CAMERA_PITCH_RADIANS,
                                Units.degreesToRadians(result.getBestTarget().getPitch()));
        
                                double yaw = result.getBestTarget().getYaw();
      System.out.println(range);
      System.out.println(yaw);
    double forwardSpeed = rangeController.calculate(range, 2);
    double turnSpeed = yawController.calculate(yaw,0);
    
      DiffTalonSubsystem.getInstance().arcadeDrive(forwardSpeed, turnSpeed);

                              }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
