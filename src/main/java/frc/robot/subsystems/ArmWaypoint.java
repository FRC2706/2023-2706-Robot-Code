// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class ArmWaypoint {

    private double x;
    private double z;

    private boolean m_isAnglesNotXY = false;

    public ArmWaypoint(double x, double z) {
        this(x, z, false);
    }

    public ArmWaypoint(double x, double z, boolean anglesNotXY) {
        this.x = x;
        this.z = z;

        if (anglesNotXY) {
            this.x = Math.toRadians(this.x);
            this.z = Math.toRadians(this.z);
        }
        m_isAnglesNotXY = anglesNotXY;
    }

    public double getX() {
        return x;
    }

    public double getZ() {
        return z;
    }

    public boolean isAnglesNotXY() {
        return m_isAnglesNotXY;
    }

}
