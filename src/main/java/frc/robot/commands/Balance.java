// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.DiffTalonSubsystem;

public class Balance extends CommandBase {
  
  PigeonIMU m_pigeon = new PigeonIMU(Config.CANID.PIGEON);
  // Servo servoCam = new Servo(0);

  double speedSlow = 0.24;
  double speedMed = 0.30;
  double rotateSpeed = 0.35;
  double rotateSpeedSlow = 0.25;
  double initialAngle = 0;

  /** Creates a new Balance. */
  public Balance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DiffTalonSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Find the reset method
    DiffTalonSubsystem.getInstance().resetPigeon();
    initialAngle = DiffTalonSubsystem.getInstance().getPitchValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(DiffTalonSubsystem.getInstance().getPitchValue());
    if (DiffTalonSubsystem.getInstance().getPitchValue() >= initialAngle - 0.05 && DiffTalonSubsystem.getInstance().getPitchValue() <= initialAngle + 0.05) {
      DiffTalonSubsystem.getInstance().arcadeDrive(0, 0);
    }
    else if (DiffTalonSubsystem.getInstance().getPitchValue() > initialAngle + 0.2) {
      if (DiffTalonSubsystem.getInstance().getPitchValue() <= initialAngle + 0.3) {
        DiffTalonSubsystem.getInstance().arcadeDrive(speedSlow, 0);
      }
      else {
        DiffTalonSubsystem.getInstance().arcadeDrive(speedMed, 0);
      }
      }
    else if (DiffTalonSubsystem.getInstance().getPitchValue() < initialAngle - 0.2) {
      if (DiffTalonSubsystem.getInstance().getPitchValue() >= initialAngle - 0.3) {
        DiffTalonSubsystem.getInstance().arcadeDrive(-speedSlow, 0);
      }
      else {
        DiffTalonSubsystem.getInstance().arcadeDrive(-speedMed, 0);
      }
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
