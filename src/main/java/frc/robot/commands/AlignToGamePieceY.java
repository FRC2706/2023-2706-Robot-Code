// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToGamePieceY extends CommandBase{
  ProfiledPIDController pidRotate = new ProfiledPIDController(2.05, 0, 0.5, 
                                        new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI));
  ProfiledPIDController pid = new ProfiledPIDController(1, 0.0, 0.2, 
                                           new TrapezoidProfile.Constraints(1, 1));
  DoubleSupplier m_supplier;
  double m_travelDistance;
  double setpoint;

  Translation2d startingLocation;

  private final double ROTATION_SETPOINT = Math.toRadians(45);

  /** Creates a new RotateAngleXY. */
  public AlignToGamePieceY(DoubleSupplier supplier, double travelDistance) {
    m_travelDistance = travelDistance;
    m_supplier = supplier;

    // pid.setTolerance(0.1);
    // pid.enableContinuousInput(-Math.PI, Math.PI);
    pidRotate.setTolerance(0.1);
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(SwerveSubsystem.getInstance());
  }

  public AlignToGamePieceY(DoubleSubscriber subscriber, double travelDistance){
    this(()-> subscriber.getAsDouble(), travelDistance);
  }

  @Override
  public void initialize() {
    startingLocation = SwerveSubsystem.getInstance().getPose().getTranslation();
    setpoint = SwerveSubsystem.getInstance().getHeading().getRadians();
  }

  @Override
  public void execute() {
    SwerveSubsystem.getInstance().drive(
        0.8, 
        calculateY(),
        calculateRot(),
        false,
        false);
  }

  public double calculateRot() {
    return(pidRotate.calculate(SwerveSubsystem.getInstance().getHeading().getRadians(), ROTATION_SETPOINT));
  }

  public double calculateY() {
    double value = m_supplier.getAsDouble();
    if (value != -99 && Math.abs(value) < 30 && 
      (Math.abs(startingLocation.getDistance(
        SwerveSubsystem.getInstance().getPose().getTranslation())) < m_travelDistance - 0.4)) {

      setpoint = SwerveSubsystem.getInstance().getPose().getY() + value * (1/30);
    }
    return(pid.calculate(SwerveSubsystem.getInstance().getPose().getY(), setpoint));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(startingLocation.getDistance(
        SwerveSubsystem.getInstance().getPose().getTranslation())) > m_travelDistance;
  }
}
