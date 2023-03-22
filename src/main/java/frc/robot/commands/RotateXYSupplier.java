// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateXYSupplier extends SwerveTeleop {

  ProfiledPIDController pid = new ProfiledPIDController(2.3, 0, 0.8, 
                                        new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI)); //pid to be tested
  DoubleSupplier m_supplier;
  double setpoint;

  /** Creates a new RotateAngleXY. */
  public RotateXYSupplier(CommandXboxController driverStick, DoubleSupplier supplier) {
    super(driverStick);

    this.m_supplier = supplier;

    pid.setTolerance(0.1);
    pid.enableContinuousInput(-Math.PI, Math.PI);
  }

  public RotateXYSupplier(CommandXboxController driverStick, DoubleSubscriber subscriber){
    this(driverStick, ()-> subscriber.getAsDouble()); 
  }

  @Override
  public void initialize() {
    super.initialize();

    setpoint = SwerveSubsystem.getInstance().getHeading().getRadians();
  }

  @Override
  public double calculateRot() {
    double value = m_supplier.getAsDouble();
    if (value != -99 && Math.abs(value) < 30) {
      setpoint = SwerveSubsystem.getInstance().getHeading().getRadians() + Math.toRadians(value*-1);
    }
    return(pid.calculate(SwerveSubsystem.getInstance().getHeading().getRadians(), setpoint));
  }
}
