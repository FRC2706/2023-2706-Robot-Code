// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ArmExtend extends CommandBase {
  private static final MotorType motorType = MotorType.kBrushless;
  private static final SparkMaxAbsoluteEncoder.Type encAbsType = SparkMaxAbsoluteEncoder.Type.kDutyCycle;

  private AbsoluteEncoder m_absoluteTopArmEncoder;
  private AbsoluteEncoder m_absoluteBottomArmEncoder;

  CANSparkMax m_topArm = new CANSparkMax(0, motorType);
  CANSparkMax m_bottomArm = new CANSparkMax(0,motorType); 

  double kP = 0;
  double kI = 0;
  double kD = 0;
  double integralMinimum = -0.5;
  double integralMaximum = 0.5;

  PIDController pid = new PIDController(kP, kI, kD);

  double NEO_GEAR_RATIO = 7.67;
  double L1 = 0; //length of arm 1
  double L2 =0; //length of arm 2
  double z = 0;//constants --> first and second z positions - depends on the height of the node we are going for
  double nodeX = 0; //the x positiion of the node we are going for
  double armSpeed = 0.2;


  /** Creates a new ArmExtend. */
  
  public ArmExtend() {

    m_absoluteTopArmEncoder = m_topArm.getAbsoluteEncoder(encAbsType);
    m_absoluteBottomArmEncoder = m_bottomArm.getAbsoluteEncoder(encAbsType);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ArmSubsystem.getInstance());
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setIntegratorRange(integralMinimum, integralMaximum);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double drivetrainx = SwerveSubsystem.getInstance().getPose().getX();
    double x = setx(drivetrainx,nodeX);
    double[] angles = calculateAngle(L1, L2, x, z);
    double angle1 = angles[0];
    double angle2 = angles[1];

    double targetAngularDistanceBottomArm = getAngularDistance(angle1, NEO_GEAR_RATIO);
    double targetAngularDistanceTopArm = getAngularDistance(angle2, NEO_GEAR_RATIO);

    if (m_absoluteBottomArmEncoder.getPosition() < targetAngularDistanceBottomArm) {
      m_bottomArm.set(pid.calculate(m_absoluteBottomArmEncoder.getVelocity(), armSpeed));
    }
    else {
      m_bottomArm.set(0);
    }


    if (m_absoluteTopArmEncoder.getPosition() < targetAngularDistanceTopArm) {
      m_topArm.set(pid.calculate(m_absoluteTopArmEncoder.getVelocity(), armSpeed));
    }
    else {
      m_topArm.set(0);
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
  public double[] calculateAngle(double L1, double L2, double x, double z) {
    double zx = (Math.pow(x,2)+Math.pow(z,2));
    double angle2 = Math.acos((zx-Math.pow(L1,2)-Math.pow(L2,2)/-2*L1*L2));
    double angle1 = Math.atan(z/x)+Math.acos((Math.pow(L2,2)-zx+Math.pow(L1,2))/-2*zx*Math.pow(L1,2));
    double[] angles = {angle1,angle2};
    return angles;
  }
  public double setx(double drivetrain_x,double Node_x){
    double x = Node_x - drivetrain_x;
    return x;
  }
  public double getAngularDistance(double angle, double gearRatio) {
    return angle * gearRatio;
  }
}
