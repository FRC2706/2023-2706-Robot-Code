package frc.robot.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlignToPoseWithOdometry;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmJoystickConeCommand;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.GripperCommand.GRIPPER_INSTRUCTION;
import frc.robot.commands.OdometryCtrl_2;
import frc.robot.config.Config;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.robotcontainers.CompRobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class DriverAidFactory {
    private final static double SCORING_X = 1.78;
    private final static double RAISE_ARM_X = SCORING_X + 0.5;
    private final static Rotation2d SCORING_ANGLE = Rotation2d.fromDegrees(180);
    private final static double VEL = 2;
    private final static double ACCEL = 2;

    public static Command scoreConeWithPid(double yCoordinateMeters, ArmSetpoint armSetpoint) {
        if (!(armSetpoint == ArmSetpoint.MIDDLE_CONE || armSetpoint == ArmSetpoint.TOP_CONE)) {
            System.out.printf("Failed to create a scoreConeWithPid since %s is not MIDDLE_CONE or TOP_CONE\n", armSetpoint.name());
            return new InstantCommand();
        }

        ArmSetpoint releaseSetpoint;
        if (armSetpoint == ArmSetpoint.TOP_CONE) {
            releaseSetpoint = ArmSetpoint.TOP_CONE_RELEASE;
        } else {
            releaseSetpoint = ArmSetpoint.MIDDLE_CONE_RELEASE;
        }

        return Commands.sequence(
            new AlignToPoseWithOdometry(
                new Pose2d(RAISE_ARM_X, yCoordinateMeters, Rotation2d.fromDegrees(180)),
                true, false, VEL, ACCEL),
            new ParallelDeadlineGroup(
                new ArmJoystickConeCommand(armSetpoint, new CommandXboxController(4)).withTimeout(2),
                new AlignToPoseWithOdometry(
                    new Pose2d(RAISE_ARM_X, yCoordinateMeters, Rotation2d.fromDegrees(180)),
                    true, true, VEL, ACCEL)
            ),
            brakes(true), 
            new AlignToPoseWithOdometry(
                new Pose2d(SCORING_X, yCoordinateMeters, Rotation2d.fromDegrees(180)),
                false, false, VEL, ACCEL).withTimeout(1),
            new ParallelCommandGroup(
                new AlignToPoseWithOdometry(
                    new Pose2d(SCORING_X, yCoordinateMeters, Rotation2d.fromDegrees(180)),
                    false, true, VEL, ACCEL),
                new WaitCommand(0.2).andThen(new ArmCommand(releaseSetpoint)),
                new WaitCommand(0.5).andThen(new GripperCommand(GRIPPER_INSTRUCTION.OPEN, CompRobotContainer.setState))
            ).withTimeout(0.8),
            brakes(true),
            new AlignToPoseWithOdometry(
                new Pose2d(RAISE_ARM_X, yCoordinateMeters, Rotation2d.fromDegrees(180)),
                false, false, VEL, ACCEL).withTimeout(1),
            new ScheduleCommand(new ArmCommand(ArmSetpoint.PICKUP))
        );
    }

    public static Command alignToConeWithPid(double yCoordinateMeters) {
        return Commands.sequence(
            new AlignToPoseWithOdometry(
                new Pose2d(RAISE_ARM_X, yCoordinateMeters, SCORING_ANGLE), 
                true, false, VEL, ACCEL),
            new AlignToPoseWithOdometry(
                new Pose2d(SCORING_X, yCoordinateMeters, SCORING_ANGLE), 
                false, false, VEL, ACCEL)      
        );
    }
    
    public static Command pathFindThenPathPlanner(BaseAutoBuilder builder, PathPlannerTrajectory traj) {
        PathPlannerState trajStartState = traj.getInitialState();
        PathPoint trajStartPoint = new PathPoint(
            trajStartState.poseMeters.getTranslation(), 
            trajStartState.poseMeters.getRotation(),
            trajStartState.holonomicRotation,
            trajStartState.velocityMetersPerSecond);
        double yCoordinateMeters = traj.getEndState().poseMeters.getY();

        return Commands.sequence(
            new OdometryCtrl_2(builder, 1, 1, trajStartPoint),
            builder.followPathWithEvents(traj),
            scoreConeWithPid(yCoordinateMeters, ArmSetpoint.MIDDLE_CONE)
        );
    }
    
            // Use alternative commands to score
            // addCommands(
            //     new OdometryCtrl_2(builder, 2.5, 3, trajStartPoint),
            //     builder.followPathWithEvents(traj),
            //     new AlignToPoseWithOdometry(finalPose, false, false, 1, 1),
            //     new ParallelDeadlineGroup(
            //         new AlignToPoseWithOdometry(finalPose, false, true, 1, 1),
            //         new ScheduleCommand(new ParallelCommandGroup(
            //             new ArmCommand(ArmSetpoint.TOP_CONE_RELEASE), // Lower arm
            //             new WaitCommand(0.2).andThen(new GripperCommand(GRIPPER_INSTRUCTION.OPEN, CompRobotContainer.setState)))) // Release gripper 
            //     ).withTimeout(0.6) // Allow 0.6 to score the before before handing control back to the driver
            // );
    

    private static Command brakes(boolean turnBrakeOn) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> ArmSubsystem.getInstance().controlBottomArmBrake(turnBrakeOn)),
            Commands.runOnce(() -> ArmSubsystem.getInstance().controlTopArmBrake(turnBrakeOn))
        );
    }
}

