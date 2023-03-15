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

public class AlignToGamePiece extends CommandBase{

  ProfiledPIDController pid = new ProfiledPIDController(2.05, 0, 0.5, 
                                        new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI));
  DoubleSupplier m_supplier;
  double m_travelDistance;
  double setpoint;

  Translation2d startingLocation;

  /** Creates a new RotateAngleXY. */
  public AlignToGamePiece(DoubleSupplier supplier, double travelDistance) {
    m_travelDistance = travelDistance;
    m_supplier = supplier;

    pid.setTolerance(0.1);
    pid.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(SwerveSubsystem.getInstance());
  }

  public AlignToGamePiece(DoubleSubscriber subscriber, double travelDistance){
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
        1.0, 
        0,
        calculateRot(),
        false,
        false);
  }

  public double calculateRot() {
    double value = m_supplier.getAsDouble();
    if (value != -99 && Math.abs(value) < 30) {
      setpoint = SwerveSubsystem.getInstance().getHeading().getRadians() + Math.toRadians(value*-1);
    }
    return(pid.calculate(SwerveSubsystem.getInstance().getHeading().getRadians(), setpoint));
  }

  @Override
  public boolean isFinished() {
    return Math.abs(startingLocation.getDistance(
        SwerveSubsystem.getInstance().getPose().getTranslation())) > m_travelDistance;
  }
}
