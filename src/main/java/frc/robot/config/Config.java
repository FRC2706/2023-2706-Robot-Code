package frc.robot.config;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Robot;

import java.io.BufferedReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * Config manager for the robot
 */
public class Config {
    
    /**
     * Instructions for set up of robot.conf file on robot
     *
     * 0. Connect to the robot to the robot using a usb cable or the wifi network.
     * 1. Using a tool like Git Bash or putty, ssh into admin@roboRIO-2706-FRC.local (ssh admin@roboRIO-2706-FRC.local)
     * a. There is no password on a freshly flashed roboRIO
     * 2. Go up a directory (cd ..)
     * 3. cd into lvuser/ (cd lvuser/)
     * 4. Create a new file called robot.conf (touch robot.conf)
     * 5. Open the file with vi (vi robot.conf)
     * 6. Press i to enter insert mode
     * 7. Add an integer denoting the robot id. If it's the first robot, use 0, second use 1 etc.
     * 8. Press [ESC] followed by typing :wq in order to save and quit
     * 9. To verify this worked type: more robot.conf
     * 10. If it displays the value you entered, it was successful
     * 11. Type exit to safely exit the ssh session
     */
    
    private static final Path ROBOT_ID_LOC = Paths.get(System.getProperty("user.home"), "robot.conf");
    
    /**
     * ID of the robot that code is running on
     */
    private static int robotId = -1;
    
    /**
     * ROBOT IDs
     * 
     * ID 0: 2023 Competition robot
     * ID 1: Clutch (Rapid React)
     * ID 2: Beetle (Small Talon tank drive)
     * ID 3: Cosmobot (Deep Space)
     * ID 4: MiniSwerve (Small swerve chassis)
     * ID 5: NeoBeetle (Small Neo tank drive)
     * ID 6: 
     * 
     *  ...
     * 
     * 
     **/

    /**
     * CAN IDs, ports, channels, etc.
     */
    public static class CANID {
        public static int DIFF_LEADER_LEFT = robotSpecific(-01, 6, 2, 5, -01, 35);
        public static int DIFF_LEADER_RIGHT = robotSpecific(-01, 3, 1, 3, -01, 33);
        public static int DIFF_FOLLOWER_LEFT = robotSpecific(-01, 5, -1, 7, -01, 37);
        public static int DIFF_FOLLOWER_RIGHT = robotSpecific(-01, 2, -1, 9, -01, 39);
    
        public static int INTAKE = robotSpecific(-01, 8, -1, -1);
        public static int SHOOTER = robotSpecific(-01, 11, 5, -1);
        public static int CLIMBER = robotSpecific(-01, 4, -1, -1);
        public static int INDEXER = robotSpecific(-01, 7, 7, -1);
    
        public static int PIGEON = robotSpecific(27, 27, 27, 27, DIFF_FOLLOWER_LEFT);
    
        public static int CANDLE = robotSpecific(-01, 15, -1, 15);
        public static int CTRE_PCM = robotSpecific(-01, 1, -1, -1);
    }
    
    public static int ANALOG_SELECTOR_PORT_ONE = robotSpecific(-01, 0, 0, -1);
    public static int ANALOG_SELECTOR_PORT_TWO = robotSpecific(-01, -1, -1, -1);
    
    /**
     * Swerve Drive Constants
     */
    
    
    /**
     * Differential Drive Constants
     */
    public static class DIFF {
        public static boolean ISNEOS = robotSpecific(true, false, false, false);
        public static boolean HAS_FOLLOWERS = robotSpecific(true, true, false, true, true);
        public static boolean LEFT_FOLLOWER_ISVICTOR = robotSpecific(false, true, false, true);
        public static boolean RIGHT_FOLLOWER_ISVICTOR = robotSpecific(false, true, false, true);
    
        // Invert motors to consider forward as forward (same practice for all objects)
        public static boolean LEADER_LEFT_INVERTED = robotSpecific(false, false, false, false, false, false);
        public static boolean LEADER_RIGHT_INVERTED = robotSpecific(false, false, true, true, false, true);
        public static boolean FOLLOWER_LEFT_INVERTED = robotSpecific(false, false, false, false, false, false);
        public static boolean FOLLOWER_RIGHT_INVERTED = robotSpecific(false, false, false, true, false, false);
    
        public static boolean LEFT_SENSORPHASE = robotSpecific(false, true, true, true);
        public static boolean RIGHT_SENSORPHASE = robotSpecific(false, false, true, true);
    
        // Current limiter Constants
        public static boolean TALON_CURRENT_LIMIT = true;   //Enable or disable motor current limiting.
        public static int TALON_PEAK_CURRENT_AMPS = 80;           //Peak current threshold to trigger the current limit
        public static int TALON_PEAK_TIME_MS = 250;               //Time after current exceeds peak current to trigger current limit
        public static int TALON_CONTIN_CURRENT_AMPS = 40;         //Current to mantain once current limit is triggered 
        
        // Drivetrain idle mode and voltage/current limits
        public static int NEO_RAMSETE_CURRENTLIMIT = 40;
        public static int NEO_DRIVER_CURRENTLIMIT = 80;

        public static IdleMode TELEOP_IDLEMODE = IdleMode.kBrake; 
        public static NeutralMode TELEOP_NEUTRALMODE = NeutralMode.Brake;

        public static IdleMode AUTO_IDLEMODE = IdleMode.kBrake; 
        public static NeutralMode AUTO_NEUTRALMODE = NeutralMode.Brake;

        public static double BRAKE_IN_DISABLE_TIME = 2.0;

        // Conversion Ratios for the SparkMax units
        public static double DRIVETRAIN_GEAR_RATIO = 10.71;
        public static double ROTATIONS_TO_METERS = Math.PI * drivetrainWheelDiameter / DRIVETRAIN_GEAR_RATIO;
        public static double RPM_TO_METERS_PER_SECOND = ROTATIONS_TO_METERS * 0.1;  

        public static double DRIVESUBSYSTEM_VOLTAGECOMP = 9;

        // P Gain -> Motor voltage to apply for every m/s of error
        public static double kRamsetePGain = 0.00082467;   
    }
    

    public static double drivetrainWheelDiameter = robotSpecific(0.1524, 0.1524, 0.1016, 0.1524, 0.1524, 0.1524); // Diameter of wheel is 0.1524
    
        
    /**
     * Clutch (Id: 1) specific constants
     */
    public static int CLUTCH_KICKER_PNEUMATIC_FORWARD_CHANNEL = 6;
    public static int CLUTCH_KICKER_PNEUMATIC_REVERSE_CHANNEL = 1;
    public static int CLUTCH_KICKER_PNEUMATIC_FLOAT_CHANNEL_1 = 0;
    public static int CLUTCH_KICKER_PNEUMATIC_FLOAT_CHANNEL_2 = 7;

    public static int CLUTCH_INTAKE_PNEUMATIC_FORWARD_CHANNEL = 4;
    public static int CLUTCH_INTAKE_PNEUMATIC_REVERSE_CHANNEL = 3;
    public static int CLUTCH_INTAKE_PNEUMATIC_FLOAT_CHANNEL_1 = 5;
    public static int CLUTCH_INTAKE_PNEUMATIC_FLOAT_CHANNEL_2 = 2;
      
    public static int CLUTCH_INDEXER_SWITCH_END = 2;
    public static int CLUTCH_INDEXER_SWITCH_MIDDLE = 3;
    public static int CLUTCH_FEEDER_SWITCH_INPUT = 9;
    public static int CLUTCH_FEEDER_SWITCH_OUTPUT = 8;
    // All Constants above are specific to Clutch
    
    /**
     * Beetle (Id: 2) specific constants
     */
    // Ringlights on the relays
    public static final String BEETLE_RELAY_NETWORKTABLE = "ControlRelay";
    public static final int BEETLE_RELAY_RINGLIGHT_REAR_SMALL = 1;
    public static final int BEETLE_RELAY_RINGLIGHT_REAR_LARGE = 2;
    public static final int BEETLE_RELAY_RINGLIGHT_FRONT = 3;

    // Sensor ports of analog inputs on Beetle
    public static final int MINIROBOT_MB1043_ANALOG_PORT = 4;
    public static final int MINIROBOT_MB1013_ANALOG_PORT = 5;
    public static final int MINIROBOT_2Y0A02_ANALOG_PORT = 6;
    public static final int MINIROBOT_0A41SK_ANALOG_PORT = 7;
    // All constants above for specific to Beetle

    
    /**
     * Common Robot Constants
     */
    public static Double DRIVER_JOYSTICK_DEADBAND = 0.1; // TODO: Investigate if this can be better tuned
        
    public static double kTrackWidth = robotSpecific(0.6, 1.2267, 0.3136, 0.569, -1.0, 0.51762);
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    public static final int DIFF_SLOTID_DRIVER = 0;
    public static final int DIFF_SLOTID_RAMSETE = 1;

    
    
    /**
     * Common Hardware Constants
     */
    public static int SRX_ENCODER_CPR = 4096;

    public static final int TALON_PRIMARY_PID = 0;
    public static final int TALTON_AUXILIARY_PID = 1;

    public static final int CAN_TIMEOUT_SHORT = 10;
    public static final int CAN_TIMEOUT_LONG = 100;

    //ultrasound MB1043/MB1013: 30cm - 500cm
    public static double MINIROBOT_MBUltraSound_RANGE_CM    = 500;
    public static double MINIROBOT_MBUltraSound_MIN_CM      = 30;
    public static double MINIROBOT_MBUltraSound_CONVERT2CM  = 0.125;
    public static double MINIROBOT_INFRARED2Y_RANGE_CM      = 150;
    public static double MINIROBOT_INFRARED2Y_MIN_CM        = 20;
    public static double MINIROBOT_INFRARED0A_RANGE_CM      = 30;
    public static double MINIROBOT_INFRARED0A_MIN_CM        = 4;


    /**
     * Math Constants
     */
    public static double METERS_IN_ONE_FOOT = 0.3048;


    /**
     * Networktables
     */
    // Define a global constants table for subsystems to use
    public static NetworkTable constantsTable = NetworkTableInstance.getDefault().getTable("constants");

    // Vision Table Constants
    public static String VISION_TABLE_NAME_CARGO = "MergeVisionPipelinePi20";
    public static String DISTANCE_CARGO         = "DistanceToCargo";
    public static String YAW_CARGO              = "CargoCentroid1Yaw";
    public static String YAW_TO_DIAMOND         = "YawToTarget";

    public static String VISION_TABLE_NAME_HUB  = "MergeVisionPipelinePi21";
    public static String DISTANCE_HUB           = "AverageDistance";
    public static String YAW_HUB                = "YawToTarget";


    /**
     * Ramsete Constants
     */
    public static double ksVolts = robotSpecific(1.1, 1.7204, 1.1848, 1.28);
    public static double kvVoltSecondsPerMeter = robotSpecific(3.03, 1.5165, 4.766, 3.13);
    public static double kaVoltSecondsSquaredPerMeter = robotSpecific(0.4, 0.72788, 1.2249, 0.463);

    public static double RAMSETE_KP = robotSpecific(0.03, 0.47993, 0.0434, 0.05);

    // Ramsete Max Velocity and max acceleration
    public static double kMaxSpeedMetersPerSecond = robotSpecific(2.4, 1.2, 1.838, 2.4);
    public static double kMaxAccelerationMetersPerSecondSquared =  robotSpecific(2.4, 1.0, 1.838, 2.4);

    
    // Scale the field
    private static double defaultScale = 1.0;
    public static double scaleField = robotSpecific(defaultScale, defaultScale, 0.7, defaultScale);

    

    // TrajectoryConfig & TrajectoryConstraint - needed to construct a trajectory
    public static TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Config.ksVolts,
            Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), Config.kDriveKinematics, 10);

    public static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint);
    

   





    /**
     * Returns one of the values passed based on the robot ID
     *
     * @param first The first value (default value)
     * @param more  Other values that could be selected
     * @param <T>   The type of the value
     * @return The value selected based on the ID of the robot
     */
    @SafeVarargs
    public static <T> T robotSpecific(T first, T... more) {
        if (getRobotId() < 1 || getRobotId() > more.length) {
            return first;
        } else {
            return more[getRobotId() - 1];
        }
    }

    /**
     * Obtain the robot id found in the robot.conf file
     *
     * @return The id of the robot
     */
    public static int getRobotId() {
        
        if (robotId < 0) {
            // Set to the ID of the 2023 Competition robot if the simulation is running
            if (RobotBase.isSimulation()) {
                return 0;

            // Not simulation so read the file on the roborio for it's robot id.
            } else {
                try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
                    robotId = Integer.parseInt(reader.readLine());
                } catch (Exception e) {
                    Robot.haltRobot("Can't load Robot ID", e);
                }
            }
        }

        return robotId;
    }
}
