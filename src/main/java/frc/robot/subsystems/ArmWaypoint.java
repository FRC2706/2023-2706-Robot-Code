// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public class ArmWaypoint {

    private double x;
    private double z;
    private double bot_vel;
    private double top_vel;

    public ArmWaypoint(double x, double z) {
        this(x, z, 0, 0);
    }

    public ArmWaypoint(double x, double z, double bottom_vel, double top_vel) {
        this.x = x;
        this.z = z;
        this.bot_vel = bottom_vel;
        this.top_vel = top_vel;

    }

    public double getBotVel() {
        return bot_vel;
    }

    public double getTopVel() {
        return top_vel;
    }

    public double getX() {
        return x;
    }

    public double getZ() {
        return z;
    }

}
