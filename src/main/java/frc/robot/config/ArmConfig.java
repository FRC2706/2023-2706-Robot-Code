// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

/** Add your docs here. */
public class ArmConfig {

    public enum ArmSetpoint {
        DEFAULT(2.467484468, 2.467484468),
        CONE_PICKUP(4, 6),
        CUBE_PICKUP(4, 4.5),
        LOW(8, 19.382),
        MEDIUM(24, 37.132),
        MEDIUM_RELEASE(24, 33.132),
        HIGH(35, 54.132),
        HIGH_RELEASE(35, 50.132);

        private double x;
        private double z;

        private ArmSetpoint(double x, double z) {
            this.x = x;
            this.z = z;
        }

        public double getX() {
            return x;
        }

        public double getZ() {
            return z;
        }
      }
    
      public static final double NEO_GEAR_RATIO = 60;
      public static final double L1 = 40; //length of arm 1
      public static final double L2 = 40; //length of arm 2
      public static final double HORIZONTAL_VOLTAGE = 1.0;

      public static final double armPositionConversionFactor = 2 * Math.PI / NEO_GEAR_RATIO;
      public static final double armVelocityConversionFactor = armPositionConversionFactor / 60.0;

      public static final double top_arm_kP = 0.8;
      public static final double top_arm_kI = 0.0005;
      public static final double top_arm_kD = 0;
      public static final double top_arm_kIz = 0.07;
      public static final double top_arm_kFF = 0;

      public static final double bottom_arm_kP = 0.8;
      public static final double bottom_arm_kI = 0.0005;
      public static final double bottom_arm_kD = 0;
      public static final double bottom_arm_kIz = 0.07;
      public static final double bottom_arm_kFF = 0;
}
