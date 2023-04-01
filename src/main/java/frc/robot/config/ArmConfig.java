// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.ArmWaypoint;

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
        // for north bay, setpoints to tune: PICKUP, BOTTOM_CONE, MIDDLE_CONE, TOP_CONE
        // if time allows, try bottom, middle, top for cubes as well

        HOME_AFTER_PICKUP(13.5, -3.5),// Default position
        HOME_WITH_GAMEPIECE(13.5, -3.5, new ArmWaypoint(19, -3)),
        
        STARTING_CONFIGURATIN(9.8, -8), // 9.8
        
        // pickup setpoint was 4.7
        PICKUP(5.0, -12.3, new ArmWaypoint(17, -3), new ArmWaypoint(8, -11)), // x was 9.7 z was -12
        PICKUP_NOWP(4.5, -12.5, new ArmWaypoint(9, -11.7)),
        PICKUP_OUTSIDE_FRAME(25, -11, new ArmWaypoint(14, -8)), // CHECK
        HUMAN_PLAYER_PICKUP(25, 35, new ArmWaypoint(14, -7)), // NOT DONE

        BOTTOM_CONE(27, -3),
        MIDDLE_CONE(33.5, 35), //, new ArmWaypoint(28, 29)), 
        MIDDLE_CONE_RELEASE(36, 28.03-7), //WRONG
        //TOP_CONE(46.5, 46.86, new ArmWaypoint(30, 20, 4, 10), new ArmWaypoint(42, 34, 3, 0.6)),
        TOP_CONE(47, 46, new ArmWaypoint(30, 20, 4, 15)),//, new ArmWaypoint(40, 30, 3, 4)), // <- This does a nice S curve if cone flipping is fixed mechanically
        TOP_CONE_NO_WAYPOINT(47, 46),
        TOP_CONE_RELEASE(53, 39-7), // WRONG

        BOTTOM_CUBE(24, -7),
        MIDDLE_CUBE(40.5, 27),//, new ArmWaypoint(21, 12, 0, 4)),
        TOP_CUBE(49, 42, new ArmWaypoint(35, 15, 0, 2.2));
        

        public DoubleEntry x_entry;
        public DoubleEntry z_entry;
        private ArmWaypoint waypoints[];

        private ArmSetpoint(double x, double z, ArmWaypoint...waypoints) {
            this.waypoints = waypoints;

            if (x < x_lower || x > x_upper || z < z_lower || z > z_upper ||
                Math.hypot(x, z) > L1 + L2
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

        public ArmWaypoint[] getWaypoint() {
            return waypoints;
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
      public static final double TOP_HORIZONTAL_VOLTAGE = 1.5;
      public static final double TOP_HORIZONTAL_VOLTAGE_CONE = 2.3;
      public static final double BOTTOM_MOMENT_TO_VOLTAGE = 0.000005;
      public static final boolean TOP_SET_INVERTED = true;
      public static final boolean BOTTOM_SET_INVERTED = true;
      public static final int CURRENT_LIMIT = 60;

      // constants for arm constraints

      // for cubes (general)
      public static final double TOP_MAX_VEL = Math.PI * 4.5;
      public static final double TOP_MAX_ACCEL = Math.PI * 4;

      // for nose in cones to middle level
      public static final double TOP_NOSE_IN_CONE_MIDDLE_MAX_VEL = Math.PI * 4.5;
      public static final double TOP_NOSE_IN_CONE_MIDDLE_MAX_ACCEL = Math.PI * 1.2;

      // for nose in cones to top level
      public static final double TOP_NOSE_IN_CONE_TOP_MAX_VEL = Math.PI * 4.5;
      public static final double TOP_NOSE_IN_CONE_TOP_MAX_ACCEL = Math.PI * 1.2;

      // for base in cones to middle level
      public static final double TOP_BASE_IN_CONE_MIDDLE_MAX_VEL = Math.PI * 4.5;
      public static final double TOP_BASE_IN_CONE_MIDDLE_MAX_ACCEL = Math.PI * 3;

       // for base in cones to top level
       public static final double TOP_BASE_IN_CONE_TOP_MAX_VEL = Math.PI * 6;
       public static final double TOP_BASE_IN_CONE_TOP_MAX_ACCEL = Math.PI * 5;

      // for cubes (bottom arm)
      public static final double BOTTOM_MAX_VEL = Math.PI * 3;
      public static final double BOTTOM_MAX_ACCEL = Math.PI *3;

      // for cones (bottom arm)
      public static final double BOTTOM_CONE_MAX_VEL = Math.PI * 3;
      public static final double BOTTOM_CONE_MAX_ACCEL = Math.PI * 2;
      

      public static final double topArmPositionConversionFactor = 2 * Math.PI / TOP_NEO_GEAR_RATIO;
      public static final double topArmVelocityConversionFactor = topArmPositionConversionFactor / 60.0;
      public static final double bottomArmPositionConversionFactor = 2 * Math.PI / BOTTOM_NEO_GEAR_RATIO;
      public static final double bottomArmVelocityConversionFactor = bottomArmPositionConversionFactor / 60.0;

      public static final double positionTolerance = Math.toRadians(1);
      public static final double velocityTolerance = Math.toRadians(1);

      public static final double waypointPositionTolerance = Math.toRadians(5);
      public static final double waypointVelocityTolerance = Math.toRadians(6);
      public static final double waypointPickupPositionTolerance = Math.toRadians(2);
      public static final double waypointPickupVelocityTolerance = Math.toRadians(2);
      public static final double waypointPickup0PositionTolerance = Math.toRadians(25);
      public static final double waypointConePositionTolerance = Math.toRadians(10);

      // PID constants for top arm (for cubes)
      public static final double top_arm_kP = 1.6;//0.23; 
      public static final double top_arm_kI = 0.002;
      public static final double top_arm_kD = 40;// 4.0;
      public static final double top_arm_kIz = 0.3;
      public static final double top_arm_kFF = 0;

      // PID constants for top arm (for cones)
      public static final double top_arm_kP2 = 3.6;//0.23; 
      public static final double top_arm_kI2 = 0.004;
      public static final double top_arm_kD2 = 80;// 4.0;
      public static final double top_arm_kIz2 = 0.35;// 0.3;
      public static final double top_arm_kFF2 = 0;

      // PID constants for bottom arm
      public static final double bottom_arm_kP = 1.4;
      public static final double bottom_arm_kI = 0.0003;
      public static final double bottom_arm_kD = 0.9;
      public static final double bottom_arm_kIz = 0.3;
      public static final double bottom_arm_kFF = 0;

      // soft limit constants for top arm
      public static final float top_arm_forward_limit = (float)Math.toRadians(190); 
      public static final float top_arm_reverse_limit = (float)Math.toRadians(7); 
      public static final boolean TOP_SOFT_LIMIT_ENABLE = true;
    
      // soft limit constants for bottom arm
      public static final float bottom_arm_forward_limit = (float)Math.toRadians(135);
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
      public static double ENCODER_SYNCING_PERIOD = 0.4; // seconds
      public static int ENCODER_SYNCING_TIMEOUT = 20; // seconds
      public static double ENCODER_SYNCING_TOLERANCE = 0.01; // radians
      public static int NUM_SYNCING_SAMPLES = 20; // num of samples needed to average

      // deadband for arm joystick
      public static double ARM_JOYSTICK_DEADBAND = 0.25;

      // debouncer
      public static final double top_brake_debounce_time = 0.1;

}
