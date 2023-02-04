// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateAngleXY extends SwerveTeleop {

  ProfiledPIDController pid = new ProfiledPIDController(5.0, 0, 0.4, 
                                        new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI)); //pid to be tested
  double angle;

  /** Creates a new RotateAngleXY. */
  public RotateAngleXY(CommandXboxController driverStick, double _angle) {
    super(driverStick);

    this.angle = _angle;

    pid.setTolerance(0.1);
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public double calculateRot() {
    return(pid.calculate(SwerveSubsystem.getInstance().getHeading().getRadians(), this.angle));
  }
}
