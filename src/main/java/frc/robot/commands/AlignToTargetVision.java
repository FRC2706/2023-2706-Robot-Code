// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.Config.Swerve;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToTargetVision extends CommandBase {

  ProfiledPIDController pidX = new ProfiledPIDController(2, 0.0, 0.2, 
                                          new TrapezoidProfile.Constraints(2, 2));
  ProfiledPIDController pidY = new ProfiledPIDController(2, 0.0, 0.2, 
                                          new TrapezoidProfile.Constraints(2, 2));
  ProfiledPIDController pidRot = new ProfiledPIDController(5.0, 0, 0.4, 
                                          new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI)); //pid to be tested

  DoubleSupplier m_visionYaw;
  DoubleSupplier m_visionDist;
  double m_visionDistanceFromTargetY;
  double m_distFromTarget;

  double xSetpoint = 0;
  double ySetpoint = 0;
  double rotSetpoint = Math.PI;

  double m_tolerance = 0.05;

  LinearFilter linearFilterX = LinearFilter.movingAverage(10);
  LinearFilter linearFilterY = LinearFilter.movingAverage(10);

  Timer time = new Timer();

  /** Creates a new AlignWithNode. */
  public AlignToTargetVision(CommandXboxController driverStick, DoubleSubscriber subscriberYaw, DoubleSubscriber subscriberDist, double distFromTarget, double tolerance) {
    this(driverStick, ()-> subscriberYaw.getAsDouble(), ()-> subscriberDist.getAsDouble(), distFromTarget, tolerance);
  }

  public AlignToTargetVision(CommandXboxController driverStick, DoubleSupplier visionYaw, DoubleSupplier visionDist, double distFromTarget, double tolerance) {
    this(driverStick, visionYaw, visionDist, distFromTarget, tolerance, 0, Math.PI);
  }
  public AlignToTargetVision(CommandXboxController driverStick, DoubleSupplier visionYaw, DoubleSupplier visionDist, double distFromTarget, double tolerance, double desiredVisionY, double desiredHeading) {

    m_visionYaw = visionYaw;
    m_visionDist = visionDist;
    m_distFromTarget = distFromTarget;
    m_visionDistanceFromTargetY = desiredVisionY;
    m_tolerance = tolerance;

    rotSetpoint = desiredHeading;

    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pidX.reset(SwerveSubsystem.getInstance().getPose().getX());
    pidY.reset(SwerveSubsystem.getInstance().getPose().getY());
    pidRot.reset(SwerveSubsystem.getInstance().getHeading().getRadians());

    //set the tolerance
    // pidX.setTolerance(0.25, 0.25);   //
    // pidY.setTolerance(0.25, 0.25);   // tolerance values TBD
    // pidRot.setTolerance(Math.PI / 4.0, Math.PI / 4.0);                    //

    pidRot.enableContinuousInput(-Math.PI, Math.PI);

    xSetpoint = SwerveSubsystem.getInstance().getPose().getX();
    ySetpoint = SwerveSubsystem.getInstance().getPose().getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double visionDistance = Units.feetToMeters(m_visionDist.getAsDouble());
    double visionHeading = Math.toRadians(m_visionYaw.getAsDouble()*-1) + SwerveSubsystem.getInstance().getHeading().getRadians() - Math.PI;
    Translation2d translate = new Translation2d(visionDistance, new Rotation2d(visionHeading));

    if (m_visionYaw.getAsDouble() != -99 && m_visionDist.getAsDouble() != -1) {
      xSetpoint = linearFilterX.calculate(SwerveSubsystem.getInstance().getPose().getX() + translate.getX() - m_distFromTarget);
      ySetpoint = linearFilterY.calculate(SwerveSubsystem.getInstance().getPose().getY() + translate.getY() - m_visionDistanceFromTargetY);
    }
    else {
      linearFilterX.reset();
      linearFilterY.reset();
    }

    double xSpeed = pidX.calculate(SwerveSubsystem.getInstance().getPose().getX(), xSetpoint);
    double ySpeed = pidY.calculate(SwerveSubsystem.getInstance().getPose().getY(), ySetpoint);
    double rot = pidRot.calculate(SwerveSubsystem.getInstance().getHeading().getRadians(), rotSetpoint);
    
    //System.out.println("X: " + SwerveSubsystem.getInstance().getPose().getX() + ", Setpoint: " + xSetpoint + ", Y: " + SwerveSubsystem.getInstance().getPose().getY() + ", Setpoint: " + ySetpoint + "Rot: " + SwerveSubsystem.getInstance().getHeading().getRadians() + ", Setpoint: " + rotSetpoint);

    SwerveSubsystem.getInstance().drive(
      xSpeed, 
      ySpeed, 
      rot, 
      true, 
      false
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    //if (pidX.atSetpoint() && pidY.atSetpoint() && pidRot.atSetpoint()) {
    if ((Math.abs(SwerveSubsystem.getInstance().getPose().getX() - xSetpoint) < m_tolerance) && (Math.abs(SwerveSubsystem.getInstance().getPose().getY() - ySetpoint) < m_tolerance) && (Math.abs(SwerveSubsystem.getInstance().getHeading().getRadians() - rotSetpoint) < Math.PI / 8.0)) {
      return(true);
    }

    return false;
  }
}
