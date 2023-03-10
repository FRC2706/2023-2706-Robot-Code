// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmDisplay extends SubsystemBase {
    private final double windowX = 140;
    private final double windowY = 85;

    private final double pivot1X = windowX/2;
    private final double pivot1Y = 15;

    private double length1;
    private double length2;

    private Mechanism2d m_mech2d;

    private DoubleJointArmDisplay m_measurementDisplay;
    private DoubleJointArmDisplay m_setpointDisplay;
  /** Creates a new ArmDisplay. */
  public ArmDisplay(double length1, double length2) {
    this.length1 = length1;
    this.length2 = length2;

    m_mech2d = new Mechanism2d(windowX, windowY);

    MechanismRoot2d m_arm1PivotPoint = m_mech2d.getRoot("ArmBumperPoint", pivot1X+13.4, pivot1Y-14.7+3);
    m_arm1PivotPoint.append(new MechanismLigament2d("ArmBumperRectangle", Units.metersToInches(1), -180, 35, new Color8Bit(Color.kBlue)));
        

    m_setpointDisplay = new DoubleJointArmDisplay(m_mech2d, 3, new Color8Bit(Color.kGreen), "Setpoint");
    m_measurementDisplay = new DoubleJointArmDisplay(m_mech2d, 12, new Color8Bit(Color.kRed), "Measurement");

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("ArmDisplay", m_mech2d);

    m_setpointDisplay.updateDisplay(Math.toRadians(70), Math.toRadians(90));
    m_measurementDisplay.updateDisplay(Math.toRadians(80), Math.toRadians(90));

  }

  private class DoubleJointArmDisplay {

    private MechanismRoot2d m_arm2PivotPoint;

    private MechanismLigament2d m_arm1Line;
    private MechanismLigament2d m_arm2Line;

    private DoubleJointArmDisplay(Mechanism2d mech2d, double lineWidth, Color8Bit color, String name) {
        MechanismRoot2d m_arm1PivotPoint = mech2d.getRoot(name+"Arm1Pivot", pivot1X, pivot1Y);
        m_arm1PivotPoint.append(new MechanismLigament2d(name+"ArmTower", 13, -90, 20, new Color8Bit(Color.kBlue)));
        
        m_arm1Line = new MechanismLigament2d(name+"Arm1Line", length1, 80, lineWidth, color);
        m_arm1PivotPoint.append(m_arm1Line);

        m_arm2PivotPoint = m_mech2d.getRoot(name+"Arm2Pivot", windowX-30, windowY-15);

        m_arm2Line = new MechanismLigament2d(name+"Arm2Line", length2, -80, lineWidth, color);
        m_arm2PivotPoint.append(m_arm2Line);
    }

    private void updateDisplay(double encoderAngle1, double encoderAngle2) {
        m_arm1Line.setAngle(Math.toDegrees(encoderAngle1));

        Translation2d deltaArm1 = new Translation2d(length1, new Rotation2d(encoderAngle1));
        Translation2d arm2Pivot = new Translation2d(pivot1X, pivot1Y).plus(deltaArm1);

        m_arm2PivotPoint.setPosition(arm2Pivot.getX(), arm2Pivot.getY());

        m_arm2Line.setAngle(Math.toDegrees(encoderAngle1 - Math.PI + encoderAngle2));
    }
  }

  public void updateMeasurementDisplay(double encoderAngle1, double encoderAngle2) {
    m_measurementDisplay.updateDisplay(encoderAngle1, encoderAngle2);
}

public void updateSetpointDisplay(double desiredAngle1, double desiredAngle2) {
    m_setpointDisplay.updateDisplay(desiredAngle1, desiredAngle2);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
