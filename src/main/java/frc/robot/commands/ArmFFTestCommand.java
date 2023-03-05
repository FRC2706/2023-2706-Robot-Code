// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ArmFFTestCommand extends CommandBase {
  private CommandXboxController m_joystick;
  private double m_maxExtraVolts;
  private boolean m_enableTop;
  private boolean m_enableBot;

  /** Creates a new ArmFFTestCommand. */
  public ArmFFTestCommand(CommandXboxController joystick, double maxExtraVolts, boolean enableBottom, boolean enableTop) {
    this.m_joystick = joystick;
    this.m_maxExtraVolts = maxExtraVolts;
    m_enableTop = enableTop;
    m_enableBot = enableBottom;
    addRequirements(ArmSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_enableBot) {
      ArmSubsystem.getInstance().setBottomArmIdleMode(IdleMode.kCoast);
    }
    if (m_enableTop) {
      ArmSubsystem.getInstance().setTopArmIdleMode(IdleMode.kCoast);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double joystickValueTop = m_joystick.getRawAxis(XboxController.Axis.kRightX.value);
    joystickValueTop = MathUtil.applyDeadband(joystickValueTop, 0.15);

    double joystickValueBottom = m_joystick.getRawAxis(XboxController.Axis.kLeftX.value);
    joystickValueBottom = MathUtil.applyDeadband(joystickValueBottom, 0.15);

    if (m_enableBot) {
      ArmSubsystem.getInstance().testFeedForwardBottom(joystickValueBottom * m_maxExtraVolts);
    }
    if (m_enableTop) {
      ArmSubsystem.getInstance().testFeedForwardTop(joystickValueTop * m_maxExtraVolts);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystem.getInstance().stopMotors();
    ArmSubsystem.getInstance().setTopArmIdleMode(IdleMode.kBrake);
    ArmSubsystem.getInstance().setBottomArmIdleMode(IdleMode.kBrake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
