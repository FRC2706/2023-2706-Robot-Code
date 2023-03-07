// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class ArmConfig {

    public static final double x_lower = 2;
    public static final double x_upper = 60;
    public static final double z_lower = -13.75;
    public static final double z_upper = 60;
    public static final double homeX = 6;
    public static final double homeZ = 8;

    public enum ArmSetpoint {
        HOME_WITH_GAMEPIECE(6, 8), //can also be used for default position
        CONE_PICKUP(8, 6, true),
        CUBE_PICKUP(8, 4.5, true),
        CONE_PICKUP_OUTSIDE(16, 6),
        CUBE_PICKUP_OUTSIDE(16, 6),
        LOW_CONE(10, 19.382),
        LOW_CUBE(10, 17.382),
        MEDIUM_CONE(24, 37.132),
        MEDIUM_CUBE(24, 35.132),
        MEDIUM_CONE_RELEASE(24, 33.132),
        HIGH_CONE(35, 54.132),
        HIGH_CUBE(35, 52.132),
        HIGH_CONE_RELEASE(35, 50.132),
        HUMAN_PLAYER_PICKUP(20, 50.132);

        private double x;
        private double z;
        private boolean slowAcceleration;


        private ArmSetpoint(double x, double z, boolean slowAcceleration) {
            this.x = x;
            this.z = z;
            this.slowAcceleration = slowAcceleration;

            if (x < x_lower || x > x_upper || z < z_lower || z > z_upper) {
                x = homeX;
                z = homeZ;
                DriverStation.reportError("Armsetpoint outside of bounds. Name of setpoint is: " + this.name(), false);
            }

        }

        private ArmSetpoint(double x, double z) {
            this(x, z, false);
        }

        public boolean getSlowAccel() {
            return slowAcceleration;
        }

        public double getX() {
            return x;
        }

        public double getZ() {
            return z;
        }
      }
      public enum ArmPosition {
        GAME_PIECE_TOP,
        GAME_PIECE_MIDDLE,
        GAME_PIECE_BOTTOM
      }
    
      public static final double TOP_NEO_GEAR_RATIO = 23.5;
      public static final double BOTTOM_NEO_GEAR_RATIO = 62.5; 
      public static final double L1 = 27.38; //length of arm 1 in inches
      public static final double L2 = 38.6; //length of arm 2 in inches
      public static final double TOP_HORIZONTAL_VOLTAGE = 0;
      public static final double BOTTOM_MOMENT_TO_VOLTAGE = 0;
      public static final boolean TOP_SET_INVERTED = false;
      public static final boolean BOTTOM_SET_INVERTED = false;
      public static final int CURRENT_LIMIT = 40;

      // constants for arm constraints
      public static final double TOP_SLOW_ACCEL_MAX_VEL = 1;
      public static final double BOTTOM_SLOW_ACCEL_MAX_VEL = 0.5;
      public static final double TOP_SLOW_ACCEL_MAX_ACCEL = Math.PI * 0.9; // radians/sec
      public static final double BOTTOM_SLOW_ACCEL_MAX_ACCEL = Math.PI * 0.4;
      public static final double TOP_MAX_VEL = 3;
      public static final double BOTTOM_MAX_VEL = 1;
      public static final double TOP_MAX_ACCEL = Math.PI * 2;
      public static final double BOTTOM_MAX_ACCEL = Math.PI * 0.7;

      public static final double RESET_ENCODER_POSITION = Math.toRadians(-90);


      public static final double topArmPositionConversionFactor = 2 * Math.PI / TOP_NEO_GEAR_RATIO;
      public static final double topArmVelocityConversionFactor = topArmPositionConversionFactor / 60.0;
      public static final double bottomArmPositionConversionFactor = 2 * Math.PI / BOTTOM_NEO_GEAR_RATIO;
      public static final double bottomArmVelocityConversionFactor = bottomArmPositionConversionFactor / 60.0;

      public static final double positionTolerance = Math.toRadians(1);
      public static final double velocityTolerance = Math.toRadians(3);

      // PID constants for top arm
      public static final double top_arm_kP = 0;
      public static final double top_arm_kI = 0;
      public static final double top_arm_kD = 0;
      public static final double top_arm_kIz = 0;
      public static final double top_arm_kFF = 0;

      // PID constants for bottom arm
      public static final double bottom_arm_kP = 0;
      public static final double bottom_arm_kI = 0;
      public static final double bottom_arm_kD = 0;
      public static final double bottom_arm_kIz = 0;
      public static final double bottom_arm_kFF = 0;

      // soft limit constants for top arm
      public static final float top_arm_forward_limit = (float)Math.toRadians(180);
      public static final float top_arm_reverse_limit = (float)Math.toRadians(30);
    
      // soft limit constants for bottom arm
      public static final float bottom_arm_forward_limit = (float)Math.toRadians(90);
      public static final float bottom_arm_reverse_limit = (float)Math.toRadians(45);

}
