// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class ArmConfig {

    public static final double x_lower = 2;
    public static final double x_upper = 60;
    public static final double z_lower = -13.75;
    public static final double z_upper = 60;
    public static final double homeX = 6;
    public static final double homeZ = 8;

    public static String m_tuningTableSetpoints = "Arm/Setpoints";

    public static NetworkTable setpointsTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableSetpoints);
    // NetworkTable setpointsTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableSetpoints); 

    public enum ArmSetpoint {
        HOME_WITH_GAMEPIECE(6.6, -11.92+2), // Default position
        PICKUP(5.5, -11.92, true),
        // CUBE_PICKUP(8.2, -9.95, true),
        PICKUP_OUTSIDE_FRAME(23.55, -9.66),
        HUMAN_PLAYER_PICKUP(21.375, 40),

        BOTTOM_CONE(21.375, -5.7),
        MIDDLE_CONE(36, 28.03),
        MIDDLE_CONE_RELEASE(36, 28.03-7),
        TOP_CONE(53, 39),
        TOP_CONE_RELEASE(53, 39-7),

        BOTTOM_CUBE(21.375, -7),
        MIDDLE_CUBE(36, 15.5),
        TOP_CUBE(53.03, 27.8);
        

        private boolean slowAcceleration;
        public DoubleEntry x_entry;
        public DoubleEntry z_entry;

        private ArmSetpoint(double x, double z, boolean slowAcceleration) {
            this.slowAcceleration = slowAcceleration;

            if (x < x_lower || x > x_upper || z < z_lower || z > z_upper ||
                Math.hypot(x, z) > L1 + L2 //66.35
            ) {
                x = homeX;
                z = homeZ;
                DriverStation.reportError("Armsetpoint outside of bounds. Name of setpoint is: " + this.name(), true);
            }

            x_entry = setpointsTuningTable.getDoubleTopic(this.name() + "X").getEntry(x);
            x_entry.accept(x);
            z_entry = setpointsTuningTable.getDoubleTopic(this.name() + "Z").getEntry(z);
            z_entry.accept(z);

        }

        private ArmSetpoint(double x, double z) {
            this(x, z, false);
        }

        public boolean getSlowAccel() {
            return slowAcceleration;
        }

        public double getX() {
            return x_entry.get();
        }

        public double getZ() {
            return z_entry.get();
        }
      }
      public enum ArmPosition {
        GAME_PIECE_TOP,
        GAME_PIECE_MIDDLE,
        GAME_PIECE_BOTTOM
      }
    
      public static final double TOP_NEO_GEAR_RATIO = Config.robotSpecific(23.5, 0.0, 0.0, 0.0, 0.0, 60.0, 60.0); //comp --> 23.5
      public static final double BOTTOM_NEO_GEAR_RATIO = 62.5;  
      public static final double L1 = 27.75; //length of arm 1 in inches
      public static final double L2 = 38.6; //length of arm 2 in inches 
      public static final double LENGTH_BOTTOM_ARM_TO_COG = 14.56;
      public static final double LENGTH_TOP_ARM_TO_COG = 28.22;
      public static final double TOP_HORIZONTAL_VOLTAGE = 1.3;
      public static final double BOTTOM_MOMENT_TO_VOLTAGE = 0.000005;
      public static final boolean TOP_SET_INVERTED = true;
      public static final boolean BOTTOM_SET_INVERTED = true;
      public static final int CURRENT_LIMIT = 40;

      // constants for arm constraints
      public static final double TOP_SLOW_ACCEL_MAX_VEL = Math.PI * 3;
      public static final double TOP_SLOW_ACCEL_MAX_ACCEL = Math.PI * 1.5; // radians/sec
      public static final double TOP_MAX_VEL = Math.PI * 12;
      public static final double TOP_MAX_ACCEL = Math.PI * 12;
      public static final double BOTTOM_SLOW_ACCEL_MAX_VEL = Math.PI;
      public static final double BOTTOM_SLOW_ACCEL_MAX_ACCEL = Math.PI * 0.5;
      public static final double BOTTOM_MAX_VEL = Math.PI * 2;
      public static final double BOTTOM_MAX_ACCEL = Math.PI * 2;

      public static final double RESET_ENCODER_POSITION = Math.toRadians(-90);


      public static final double topArmPositionConversionFactor = 2 * Math.PI / TOP_NEO_GEAR_RATIO;
      public static final double topArmVelocityConversionFactor = topArmPositionConversionFactor / 60.0;
      public static final double bottomArmPositionConversionFactor = 2 * Math.PI / BOTTOM_NEO_GEAR_RATIO;
      public static final double bottomArmVelocityConversionFactor = bottomArmPositionConversionFactor / 60.0;

      public static final double positionTolerance = Math.toRadians(2);
      public static final double velocityTolerance = Math.toRadians(5);

      // PID constants for top arm
      public static final double top_arm_kP = 0.23; 
      public static final double top_arm_kI = 0.0001;
      public static final double top_arm_kD = 0;
      public static final double top_arm_kIz = 0.3;
      public static final double top_arm_kFF = 0;

      // PID constants for bottom arm
      public static final double bottom_arm_kP = 0.900000;
      public static final double bottom_arm_kI = 0.0;
      public static final double bottom_arm_kD = 0.900000;
      public static final double bottom_arm_kIz = 0;
      public static final double bottom_arm_kFF = 0;

      // soft limit constants for top arm
      public static final float top_arm_forward_limit = (float)Math.toRadians(190); 
      public static final float top_arm_reverse_limit = (float)Math.toRadians(15); 
      public static final boolean TOP_SOFT_LIMIT_ENABLE = true;
    
      // soft limit constants for bottom arm
      public static final float bottom_arm_forward_limit = (float)Math.toRadians(95);
      public static final float bottom_arm_reverse_limit = (float)Math.toRadians(40);
      public static final boolean BOTTOM_SOFT_LIMIT_ENABLE = true;

      // ff calculation for bottom arm
      public static final double gravitationalConstant = 389.0886; // inches/s/s  which is equal to 9.81 m/s/s
      public static final double BOTTOM_ARM_FORCE = 11.29 * gravitationalConstant; // 11.29 lb
      public static final double TOP_ARM_FORCE = 7.77 * gravitationalConstant; // 7.77 lb
      public static final double CONE_ARM_FORCE = 1.21 * gravitationalConstant; // 1.21 lb

      // pid controller constants
      public static final double min_output = -1;
      public static final double max_output = 1;

      // duty cycle encoders
      public static final int top_duty_cycle_channel = 9;
      public static final int bottom_duty_cycle_channel = 7;

      // arm offsets
      public static final double top_arm_offset = -282.000000;
      public static final double bottom_arm_offset = 307.800000;

      // Syncing encoders
      public static double ENCODER_SYNCING_PERIOD = 0.2; // seconds
      public static int ENCODER_SYNCING_TIMEOUT = 20; // seconds
      public static double ENCODER_SYNCING_TOLARANCE = 0.008; // radians
      public static int NUM_SYNCING_SAMPLES = 20; // num of samples needed to average

}
