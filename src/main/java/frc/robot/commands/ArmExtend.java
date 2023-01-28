// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import javax.security.auth.x500.X500Principal;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ArmExtend extends CommandBase {
  private static final MotorType motorType = MotorType.kBrushless;


  /** Creates a new ArmExtend. */
  
  CANSparkMax m_topArm = new CANSparkMax(0, motorType);
  CANSparkMax m_bottomArm = new CANSparkMax(0,motorType);
  
  public ArmExtend() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    double L1 = 0;
    double L2 =0;

    double drivetrainx = SwerveSubsystem.getInstance().getPose().getX();
    double nodeX = 0;
    double x = setx(drivetrainx,nodeX);
    double z = 0;//constants --> first and second z positions
    double[] angles = calculateAngle(L1, L2, x, z);
    double angle1 = angles[0];
    double angle2 = angles[1];

    double gearRatio = 7.67;
    double targetAngularSpeedBottomArm = angle1/gearRatio;
    double targetAngularSpeedTopArm = angle2/gearRatio;


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  public double[] calculateAngle(double L1, double L2, double x, double z) {
    double zx = (Math.pow(x,2)+Math.pow(z,2));
    double angle2 = Math.cosh((zx-Math.pow(L1,2)-Math.pow(L2,2)/-2*L1*L2));
    double angle1 = Math.tanh(z/x)+Math.cosh((Math.pow(L2,2)-zx+Math.pow(L1,2))/-2*zx*Math.pow(L1,2));
    double[] angles = {angle1,angle2};
    return angles;
  }
  public double setx(double drivetrain_x,double Node_x){
    double x = Node_x - drivetrain_x;
    return x;
  }
}
