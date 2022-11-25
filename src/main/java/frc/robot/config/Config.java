package frc.robot.config;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.Robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.FileHandler;
import java.util.logging.SimpleFormatter;

import com.ctre.phoenix.ErrorCode;

/**
 * Config manager for the robot
 */
public class Config {

    public static final double FL_ENCODER_OFFSET = -(155 + 180)-3.22;
    public static final double FR_ENCODER_OFFSET = -(94 + 180);
    public static final double RL_ENCODER_OFFSET = -(200 + 180)-0.08;
    public static final double RR_ENCODER_OFFSET = -(135 + 180)-1.24;
    public static final double kWheelBase = 0.7;
    public static double kTrackWidth = robotSpecific(-01);


    public static final int CANID_FRONT_LEFT_DRIVE = 6;
    public static final int CANID_REAR_LEFT_DRIVE = 0;
    public static final int CANID_FRONT_RIGHT_DRIVE = 0;
    public static final int CANID_REAR_RIGHT_DRIVE = 0;

    public static final int CANID_FRONT_LEFT_STEERING = 5;
    public static final int CANID_REAR_LEFT_STEERING = 0;
    public static final int CANID_FRONT_RIGHT_STEERING = 0;
    public static final int CANID_REAR_RIGHT_STEERING = 0;

    public static final int CANID_FRONT_LEFT_CANCODER = 0;
    public static final int CANID_REAR_LEFT_CANCODER = 0;
    public static final int CANID_FRONT_RIGHT_CANCODER = 0;
    public static final int CANID_REAR_RIGHT_CANCODER = 0; 

    public static final double turningEncoderConstant = (2*Math.PI)/8.0;
    public static final double drivetrainEncoderConstant = 0.1016*Math.PI*(1/(60*7.615));

    public static final double drive_kIZone = 0.15;
    public static final double drive_kFF = 0.25; // These can also be module specific.
    public static final double drive_kP = 0.2; // Hopefully they won't need to be.
    public static final double drive_kI = 0.002; // Depends on hardware differences.
    public static final double drive_kD =1.0;
	public static FluidConstant<Double> fluid_drive_kFF = new FluidConstant<>("Drive kFF", drive_kFF, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

    public static FluidConstant<Double> fluid_drive_kP = new FluidConstant<>("Drive kP", drive_kP, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

	public static FluidConstant<Double> fluid_drive_kI = new FluidConstant<>("Drive kI", drive_kI, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));


	public static FluidConstant<Double> fluid_drive_kD = new FluidConstant<>("Drive kD", drive_kD, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

	public static FluidConstant<Double> fluid_drive_kIZone = new FluidConstant<>("Drive kIZone", drive_kI, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));


    public static final double steering_kFF = 0;
    public static final double steering_kP = 0.8;
    public static final double steering_kI = 0.016;
    public static final double steering_kD = 1.6;
    public static final double steering_kIZone = 0.05; //5 degrees
    public static FluidConstant<Double> fluid_steering_kFF = new FluidConstant<>("Steering kFF", steering_kFF, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

    public static FluidConstant<Double> fluid_steering_kP = new FluidConstant<>("Steering kP", steering_kP, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

    public static FluidConstant<Double> fluid_steering_kI = new FluidConstant<>("Steering kI", steering_kI, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

    public static FluidConstant<Double> fluid_steering_kD = new FluidConstant<>("Steering kD", steering_kD, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));
                    
    public static FluidConstant<Double> fluid_steering_kIZone = new FluidConstant<>("Steering kIZone", steering_kI, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));
    // Distance between centers of right and left wheels on robot

    public static final SwerveDriveKinematics kSwerveDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static int CAN_PIGEON = 27;

    public static final double kMaxAttainableWheelSpeed = 3.0;
    public static final double kMaxAutoSpeed = 3; // m/s
    public static final double kMaxAutoAcceleration = 3; // m/s/s
    public static final double kMaxAutoAngularSpeed = Math.PI; // rad/s
    public static final double kMaxAutoAngularAcceleration = Math.PI; // rad/s/s

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAutoAngularSpeed, kMaxAutoAngularAcceleration);

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
    
    public static FileHandler logFileHandler;
    
    static {
        try {
            String logFilename = new SimpleDateFormat("'Robotlog_'yyyy'-'MM'-'dd'_'HH'-'mm'-'ss'.txt'").format(new Date());
            logFileHandler = new FileHandler("/home/lvuser/logs/" + logFilename);
            SimpleFormatter formatter = new SimpleFormatter();
            logFileHandler.setFormatter(formatter);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    /**
     * ID of the robot that code is running on
     */
    public static int robotId = -1;
    
    /**
     * ROBOT IDs
     * 
     * ID 0: 2023 Competition robot
     * ID 1: Clutch (Rapid React)
     * ID 2: Beetle (Small Talon tank drive)
     * ID 3: Cosmobot (Deep Space)
     * ID 4:
     * 
     *  ...
     * 
     * 
     **/

    
    // This is a static class which should not be instantiated
    private Config() {
    
    }

    /**
     * CAN IDs, ports, channels, etc.
     */
    public static int DIFF_FRONT_LEFT_CANID = robotSpecific(-01, 6, 2, 5, 1);
    public static int DIFF_FRONT_RIGHT_CANID = robotSpecific(-01, 3, 1, 3, 2);
    public static int DIFF_REAR_LEFT_CANID = robotSpecific(-01, 5, -1, 7, 3);
    public static int DIFF_REAR_RIGHT_CANID = robotSpecific(-01, 2, -1, 9, 4);

    public static int INTAKE_CANID = robotSpecific(-01, 8, -1, -1, -1);
    public static int SHOOTER_CANID = robotSpecific(-01, 11, 5, -1, 16); //Beetle prototype on Beetle:5
    public static int CLIMBER_CANID = robotSpecific(-01, 4, -1, -1, 16);
    public static int INDEXER_CANID = robotSpecific(-01, 7, 7, -1); //Beetle prototype on Beetle:7

    public static int TALON_5_PLYBOY = robotSpecific(-1, -1, -1, -1, -1, 5);

    public static int PIGEON_ID = robotSpecific(CLIMBER_CANID, 27, 27, 27, DIFF_REAR_LEFT_CANID, TALON_5_PLYBOY);

    public static int CANDLE_CANID = robotSpecific(-01, 15, -1, 15);
    public static int CTRE_PCM_CANID = robotSpecific(-01, 1, -1, -1);

    public static int ANALOG_SELECTOR_PORT_ONE = robotSpecific(-01, 0, 0, -1);
    public static int ANALOG_SELECTOR_PORT_TWO = robotSpecific(-01, -1, -1, -1, -1);
    
    /**
     * Differential Drive Constants
     */
    //
    public static boolean DIFF_ISNEOS = robotSpecific(true, false, false, false);
    public static boolean DIFF_LEFT_FOLLOWER_ISVICTOR = robotSpecific(true, true, false, true);
    public static boolean DIFF_RIGHT_FOLLOWER_ISVICTOR = robotSpecific(true, true, false, true);

    // Invert motors to consider forward as forward (same practice for all objects)
    public static boolean DIFF_FRONT_LEFT_INVERTED_ = robotSpecific(false, false, false, false);
    public static boolean DIFF_FRONT_RIGHT_INVERTED = robotSpecific(false, false, true, true);
    public static boolean DIFF_REAR_LEFT_INVERTED = robotSpecific(false, true, false, false);
    public static boolean DIFF_REAR_RIGHT_INVERTED = robotSpecific(false, false, false, true);

    public static boolean INVERTED_FRONT_LEFT_DRIVE = robotSpecific(true);
    public static boolean INVERTED_REAR_LEFT_DRIVE =  robotSpecific(true);
    public static boolean INVERTED_FRONT_RIGHT_DRIVE = robotSpecific(true);
    public static boolean INVERTED_REAR_RIGHT_DRIVE = robotSpecific(true);

    public static boolean INVERTED_FRONT_LEFT_STEERING =  robotSpecific(true);
    public static boolean INVERTED_REAR_LEFT_STEERING =  robotSpecific(true);
    public static boolean INVERTED_FRONT_RIGHT_STEERING = robotSpecific(true);
    public static boolean INVERTED_REAR_RIGHT_STEERING =  robotSpecific(true);

    public static boolean DIFF_LEFT_SENSORPHASE = robotSpecific(false, true, true, true);
    public static boolean DIFF_RIGHT_SENSORPHASE = robotSpecific(false, false, true, true);

    // Current limiter Constants
    public static boolean DIFF_MOTOR_CURRENT_LIMIT = true;   //Enable or disable motor current limiting.
    public static int DIFF_TALON_PEAK_CURRENT_AMPS = 80;           //Peak current threshold to trigger the current limit
    public static int DIFF_TALON_PEAK_TIME_MS = 250;               //Time after current exceeds peak current to trigger current limit
    public static int DIFF_TALON_CONTIN_CURRENT_AMPS = 40;         //Current to mantain once current limit is triggered 
    
    // Drivetain data
    public static double drivetrainWheelDiameter = robotSpecific(0.1524, 0.1524, 0.1016, 0.1524, 0.1524, 0.1524); //     of wheel is 0.1524
    public static int ticksPerRevolution = 4096;
    
        
    /**
     * Clutch (Id: 1) specific constants
     */
    public static int KICKER_PNEUMATIC_FORWARD_CHANNEL = robotSpecific(-1, 6, -1, -1);
    public static int KICKER_PNEUMATIC_REVERSE_CHANNEL = robotSpecific(-1, 1, -1, -1);
    public static int KICKER_PNEUMATIC_FLOAT_CHANNEL_1 = robotSpecific(-1, 0, -1, -1);
    public static int KICKER_PNEUMATIC_FLOAT_CHANNEL_2 = robotSpecific(-1, 7, -1, -1);

    public static int INTAKE_PNEUMATIC_FORWARD_CHANNEL = robotSpecific(-1, 4, -1, -1);
    public static int INTAKE_PNEUMATIC_REVERSE_CHANNEL = robotSpecific(-1, 3, -1, -1);
    public static int INTAKE_PNEUMATIC_FLOAT_CHANNEL_1 = robotSpecific(-1, 5, -1, -1);
    public static int INTAKE_PNEUMATIC_FLOAT_CHANNEL_2 = robotSpecific(-1, 2, -1, -1); //not used
      
        
    



    public static int INDEXER_SWITCH_END = robotSpecific(-1,2,8,-1);//Beetle prototype
    public static int INDEXER_SWITCH_MIDDLE = robotSpecific(-1,3,8,-1);
    public static int INDEXER_SWITCH = robotSpecific(-1,2,8,-1);//Beetle prototype
    public static int FEEDER_SWITCH_INPUT = robotSpecific(9, -1, -1, -1);
    public static int FEEDER_SWITCH_OUTPUT = robotSpecific(8, -1, -1, -1);
    public static int FEEDER_MAX_BALLS = 3;
    public static int FEEDERSUBSYSTEM_INDEX_ALLOWABLE_ERROR = 50; 
    public static int FEEDERSUBSYSTEM_POS_PAST_SWITCH = 800;

    public static double FEEDER_MM_CRUISE_VELOCITY = 1500;
    public static double FEEDER_MM_ACCELERATION = 2000;
    public static int FEEDER_MM_SCURVE = 2;

    public static double FEEDERSUBSYSTEM_ARBFF_ONE = 0.13;
    public static double FEEDERSUBSYSTEM_ARBFF_TWO = 0.17;
    public static double FEEDERSUBSYSTEM_ARBFF_THREE = 0.20;
    public static double FEEDERSUBSYSTEM_ARBFF_FOUR = 0.20;
    
    public static Double JOYSTICK_AXIS_DEADBAND = 0.1;
    
    public static int LEFT_CONTROL_STICK_Y = 1;
    public static int LEFT_CONTROL_STICK_X = 0;
    
    public static int RIGHT_CONTROL_STICK_Y = 5;
    public static int RIGHT_CONTROL_STICK_X = 4;
    
    public static boolean INVERT_FIRST_AXIS = robotSpecific(true, true, true);
    public static boolean INVERT_SECOND_AXIS = robotSpecific(false, false, false);
    

    
    /**
     * Common Constants
     */
    public static final int CAN_TIMEOUT_SHORT = 10;
    public static final int CAN_TIMEOUT_LONG = 100;

    public static final int TALON_PRIMARY_PID = 0;
    public static final int TALTON_AUXILIARY_PID = 1;

    public static final int DIFF_SLOTID_DRIVER = 0;
    public static final int DIFF_SLOTID_RAMSETE = 1;

    public static double METERS_IN_ONE_FOOT = 0.3048;
    
  
    // EVERYTHING ABOVE IS CHECKED
    // EVERYTHING BELOW IS UNCHECKED
    
    /**
     * Networktables
     */
    // Define a global constants table for subsystems to use
    public static NetworkTable constantsTable = NetworkTableInstance.getDefault().getTable("constants");

    // Vision Table Constants
    //@todo: double check names of network table entry
    public static String VISION_TABLE_NAME_CARGO = "MergeVisionPipelinePi20";
    public static String DISTANCE_CARGO         = "DistanceToCargo";
    public static String YAW_CARGO              = "CargoCentroid1Yaw";
    public static String YAW_TO_DIAMOND         = "YawToTarget";

    public static String VISION_TABLE_NAME_HUB  = "MergeVisionPipelinePi21";
    public static String DISTANCE_HUB           = "AverageDistance";
    public static String YAW_HUB                = "YawToTarget";

    // Drivetrain PID values
    public static double DRIVETRAIN_P_SPECIFIC = robotSpecific(0.037, 0.037, 0.018, 0.018d, 0.0, 0.25);
    public static double DRIVETRAIN_D_SPECIFIC = robotSpecific(0.0023, 0.0023, 0.0016, 0.0016d, 0.0, 0.03);

    // Frc-characterization data
    // id0: CompBot 
    // id1: CompBot(Rapid React) - tuned on limited area of carpet in unit 105. 
    //                             may need more tuning in the bigger competition carpet.
    // id2: Beetle - church parking lot 1.32, 4.65, 0.5; 
    //              - Turkish Center Carpet: 1.0251, 3.7758, 0.72224
    //              - Competition Carpet: 1.1848, 4.766, 1.2249 (forward/backward)
    //              - Competition Carpet: 2.5335, 3.0167, 1.2117 (rotation)
    // id3: Cosmobot - scaled robot. Not charaterized yet.
    public static double ksVolts = robotSpecific(1.1, 1.7204, 1.1848, 1.28);
    public static double kvVoltSecondsPerMeter = robotSpecific(3.03, 1.5165, 4.766, 3.13);
    public static double kaVoltSecondsSquaredPerMeter = robotSpecific(0.4, 0.72788, 1.2249, 0.463);

    // Track width and kinematics
    // id2: Beetle on competition carpet 0.34928
    //                parking lot: 0.3136
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

    // Ramsete Max Velocity and max acceleration
    //Beetle: old value = 1.5; new value = 1.838
    public static double kMaxSpeedMetersPerSecond = robotSpecific(2.4,1.2,1.838,2.4);
    public static double kMaxAccelerationMetersPerSecondSquared =  robotSpecific(2.4,1.0,1.838,2.4);

    public static double kRamseteTransferSpeed = kMaxSpeedMetersPerSecond;
    public static double kRamseteTurnAroundSpeed = kMaxSpeedMetersPerSecond; 
    public static double kRamseteBounceEndSpeed = kMaxSpeedMetersPerSecond-0.3;
    public static double kRamseteGalacticSpeed = kMaxSpeedMetersPerSecond-0.7;

    
    // Scale the field
    private static double defaultScale = 1.0;
    public static double scaleField = robotSpecific(defaultScale, defaultScale, 1.0, defaultScale);

    // VISION STUFF BELOW
    // Allowable vision error in meters
    public static double ALLOWABLE_VISION_ODOMETRY_ERROR = 0.5;

    // Vision code that means no target found
    public static double VISION_NO_TARGET_CODE = -99;

    // Change the side of the vision data for each type of angle
    // The desired is to have these as 1 but they are here as backup in case
    // Put either 1 or -1 which gets multiplied by the angle
    public static byte VISION_FLIP_ANGLE = 1;
    public static byte VISION_FLIP_PERPENDICULAR_ANGLE = 1;

    // Set whether to use the vision perpendicular angle or the gyro to figure out 
    // the rotation at a vision target
    public static boolean useVisionPerpendicularAngle = true;
    
    // If camera is facing backwards then put rotation as 180 degrees
    // The location of the camera from the centre of the robot
    public static Pose2d middleOfConesCameraLocation = robotSpecific(
                                            new Pose2d(), 
                                            new Pose2d(), 
                                            new Pose2d(),
                                            new Pose2d());

    // The location of the camera from the centre of the robot
    public static Pose2d diamondTapeCamera = robotSpecific(
                                                new Pose2d(), 
                                                new Pose2d(), 
                                                new Pose2d(),
                                                new Pose2d(0.3, -0.174, Rotation2d.fromDegrees(0)));

    public static final String RELAY_NETWORKTABLE = "ControlRelay";
    public static final int RELAY_RINGLIGHT_REAR_SMALL = 1; // NUMBERS NOT ACCURATELY RELATED TO CAMERAS YET
    public static final int RELAY_RINGLIGHT_REAR_LARGE = 2;
    public static final int RELAY_RINGLIGHT_FRONT = 3;


    // TrajectoryConfig & TrajectoryConstraint - needed to construct a trajectory
    public static TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Config.ksVolts,
            Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), Config.kDriveKinematics, 10);

    public static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(kDriveKinematics).addConstraint(autoVoltageConstraint);
    
    
    // Ramsete/Talon P values
    // P Values from characterization:
    // id0: 
    // id1:
    // id2: church parking lot: kp = 0.0434
    //      Turkish center carpet: forward/backward kp = 1.0805, rotation kp = 1.3199
    //      Competition carpet: forward/backward kp = 2.2772, rotation kp = 2.2083
    //      tuned for competition carpet: kF=0.38; kP=2.3; kI = 0; kD= 0.03. Works perfectly.
    // id3: 0.0888 from church parking lot, 0.0105 from basement -> averaged to 0.05 (idk but it worked)
    // kF:
    // id2: (Beetle) based on 75% output
    public static int DRIVETRAIN_SLOTID_RAMSETE = 1;
    public static double RAMSETE_KF = robotSpecific(0.0, 0.0, 0.38, 0.0);
    public static double RAMSETE_KP = robotSpecific(0.03, 0.47993, 0.0434, 0.05); //rapid: 2.381
    public static double RAMSETE_KI = 0;
    public static double RAMSETE_KD = 0; //maybe set to some value
    public static double RAMSETE_ALLOWABLE_PID_ERROR = 0; // <- never stop the P loop from running
    public static double RAMSETE_VOLTAGE_COMPENSATION = 12;

    public static int DRIVETRAIN_SLOTID_ALIGNMENT = 2;
    public static double ALIGNMENT_KF = robotSpecific(0.0, 0.343, 0.38, 0.0);
    public static double ALIGNMENT_KP = robotSpecific(0.0, 0.15153, 2.3, 0.0);//sysId for RapidReact: 0.0027242;
    public static double ALIGNMENT_KI = 0;
    public static double ALIGNMENT_KD = 0.03; 
    public static double ALIGNMENT_ALLOWABLE_PID_ERROR = 0; // <- never stop the P loop from running


    public static final FluidConstant<Integer> RPM = new FluidConstant<>("Shooter RPM", 1700)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> DRIVETRAIN_P = new FluidConstant<>("DrivetrainP", DRIVETRAIN_P_SPECIFIC)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> DRIVETRAIN_D = new FluidConstant<>("DrivetrainD", DRIVETRAIN_D_SPECIFIC)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> maxTimeOuterPortCommand = new FluidConstant<>("Outer Port Max Time", 1.0)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> maxYawErrorOuterPortCommand = new FluidConstant<>("Outer Port Command Yaw Error", 3.0)
            .registerToTable(Config.constantsTable);
    
    // PID Values for the DrivetrainPIDTurnDelta command
    public static FluidConstant<Double> PIDTURNDELTA_P = new FluidConstant<>("DrivetrainP", 0.018d)
            .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> PIDTURNDELTA_D = new FluidConstant<>("DrivetrainD", 0.0016d)
            .registerToTable(Config.constantsTable);

    // Fluid constant for Drivetrains
    public static FluidConstant<Double> DRIVETRAIN_SENSITIVE_MAX_SPEED = new FluidConstant<>("DrivetrainSensitiveMaxSpeed", 0.2)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> DRIVETRAIN_DEFAULT_MAX_SPEED = new FluidConstant<>("DrivetrainDefaultMaxSpeed", 1.0)
            .registerToTable(Config.constantsTable);

    public static FluidConstant<Double> FEEDERSUBSYSTEM_INCREMENT_TICKS = new FluidConstant<>("IncrementTicks", 13000.0)
            .registerToTable(Config.constantsTable);
    //Max distance at which the robot knows a ball is at the indexer
    public static FluidConstant<Integer> FEEDERSUBSYSTEM_IR_MAX_DISTANCE = new FluidConstant<>("IrMaxDistance", 0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_P = new FluidConstant<>("FeederSubsystemP", 0.8)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_I = new FluidConstant<>("FeederSubsystemI", 0.001)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_D = new FluidConstant<>("FeederSubsystemD", 8.0)
                .registerToTable(Config.constantsTable);
    public static FluidConstant<Double> FEEDERSUBSYSTEM_F = new FluidConstant<>("FeederSubsystemF", 0.2045)
                .registerToTable(Config.constantsTable);
    //Highest speed the motor could reach
    public static FluidConstant<Double> FEEDERSUBSYSTEM_PEAK_OUTPUT = new FluidConstant<>("FeederSubsystemPeakOutput", 0.35)
                .registerToTable(Config.constantsTable);

    public static FluidConstant<Integer> FEEDERSUBSYSTEM_IZONE = new FluidConstant<>("FeederSubsystemIZONE", 120)
                .registerToTable(Config.constantsTable);

    public static double DRIVETRAIN_SENSITIVE_FORWARD_SPEED = 0.5;
    public static double DRIVETRAIN_SENSITIVE_ROTATE_SPEED = 0.2;
    
    //Sensor ports of analog inputs on Beetle
    public static final int MINIROBOT_MB1043_ANALOG_PORT = 4;
    public static final int MINIROBOT_MB1013_ANALOG_PORT = 5;
    public static final int MINIROBOT_2Y0A02_ANALOG_PORT = 6;
    public static final int MINIROBOT_0A41SK_ANALOG_PORT = 7;

    //ultrasound MB1043/MB1013: 30cm - 500cm
    public static double MINIROBOT_MBUltraSound_RANGE_CM    = 500;
    public static double MINIROBOT_MBUltraSound_MIN_CM      = 30;
    public static double MINIROBOT_MBUltraSound_CONVERT2CM  = 0.125;
    public static double MINIROBOT_INFRARED2Y_RANGE_CM      = 150;
    public static double MINIROBOT_INFRARED2Y_MIN_CM        = 20;
    public static double MINIROBOT_INFRARED0A_RANGE_CM      = 30;
    public static double MINIROBOT_INFRARED0A_MIN_CM        = 4;


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
    private static int getRobotId() {
        if (robotId < 0) {
            try (BufferedReader reader = Files.newBufferedReader(ROBOT_ID_LOC)) {
                robotId = Integer.parseInt(reader.readLine());
            } catch (Exception e) {
                Robot.haltRobot("Can't load Robot ID", e);
            }
        }
        return robotId;
    }
    
    /**
     *
     * @param value The raw axis value from the control stick
     * @return The filtered value defined by the acceptable dead band
     */
    public static double removeJoystickDeadband(double value) {
        if (value <= JOYSTICK_AXIS_DEADBAND && value >= 0) {
            return 0;
        } else if (value >= -JOYSTICK_AXIS_DEADBAND && value <= 0) {
            return 0;
        } else {
            return value;
        }
    }
}
