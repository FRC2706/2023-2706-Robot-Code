package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GripperCommand.GRIPPER_INSTRUCTION;
import frc.robot.config.ArmConfig.ArmSetpoint;
import frc.robot.robotcontainers.CompRobotContainer;

public class FullDriverAid extends SequentialCommandGroup{
    public FullDriverAid(BaseAutoBuilder builder, PathPlannerTrajectory traj, Pose2d finalPose) {
        PathPlannerState trajStartState = traj.getInitialState();
        PathPoint trajStartPoint = new PathPoint(
            trajStartState.poseMeters.getTranslation(), 
            trajStartState.poseMeters.getRotation(),
            trajStartState.holonomicRotation,
            trajStartState.velocityMetersPerSecond);


        // Rely on PathPlanner to score
        addCommands(
            new OdometryCtrl_2(builder, 2.5, 3, trajStartPoint),
            builder.followPathWithEvents(traj)
        );


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
    }
}
