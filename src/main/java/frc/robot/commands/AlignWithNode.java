// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignWithNode extends CommandBase {

  PIDController pidX = new PIDController(0, 0, 0);   // 
  PIDController pidY = new PIDController(0, 0, 0);   //  PID values to be tested
  PIDController pidRot = new PIDController(0, 0, 0); // 

  double setpointX = 1.0;
  //double setpointY = 1.0;
  double setpointRot = 180.0;

  DoubleSupplier m_visionYaw;
  DoubleSupplier m_visionDist;
  double m_distFromNode;

  double xSpeed = 0;
  double ySpeed = 0;
  double rot = 0;

  Timer time = new Timer();

  /** Creates a new AlignWithNode. */
  public AlignWithNode(CommandXboxController driverStick, DoubleSupplier visionYaw, DoubleSupplier visionDist, double distFromNode) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_visionYaw = visionYaw;
    m_visionDist = visionDist;
    m_distFromNode = distFromNode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set the tolerance
    pidX.setTolerance(0, 0);   //
    pidY.setTolerance(0, 0);   // tolerance values TBD
    pidRot.setTolerance(0, 0); //
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_visionYaw.getAsDouble() != -99 && m_visionDist.getAsDouble() != -99) {
      xSpeed = pidX.calculate(m_visionDist.getAsDouble(), m_distFromNode);
      ySpeed = pidY.calculate(m_visionYaw.getAsDouble(), 0);
      rot = pidRot.calculate(SwerveSubsystem.getInstance().getHeading().getDegrees(), setpointRot);
    }
    
    SwerveSubsystem.getInstance().drive(
      xSpeed, 
      ySpeed, 
      rot, 
      false, 
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (pidX.atSetpoint() && pidY.atSetpoint() && pidRot.atSetpoint()) {
      return(true);
    }
    return false;
  }
}
