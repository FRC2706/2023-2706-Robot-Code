package frc.robot;

import java.util.Scanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class test {
    static final double loopPeriodSecs = 0.02;

    public static void main(String[] args) {

        Scanner in = new Scanner(System.in);
        while (true) {
            String x = getValueFromUser(in, "Enter the X speed (q to quit):");
            if (x == "q") {
                break;
            } else if (x == "failed") {
                continue;
            }

            String y = getValueFromUser(in, "Enter the Y speed (q to quit):");
            if (y == null) {
                continue;
            } else if (x == "failed") {
                continue;
            }

            String rot = getValueFromUser(in, "Enter the rot speed (q to quit):");
            if (rot == null) {
                continue;
            } else if (x == "failed") {
                continue;
            }

            ChassisSpeeds speeds;
            try {
                double xSpeed = Double.parseDouble(x);
                double ySpeed = Double.parseDouble(y);
                double rotSpeed = Double.parseDouble(rot);
                speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
            } catch (Exception e) {
                System.out.println("Ultra failed");
                break;
            }

            System.out.printf("\n\nStarting Speeds: %s\nAdjusted Speeds:\n%s\n\n\n",
                speeds.toString(),
                calc(speeds).toString()
            );

        }

        in.close();

    }

    public static String getValueFromUser(Scanner in, String msg) {
        System.out.println(msg);
        String s = in.nextLine();
        if (s.toLowerCase() == "q") {
            return "q";
        }

        try {
            double val = Double.parseDouble(s);
            System.out.println("You entered " + val);
        } catch(Exception e) {
            System.out.println("Failed to create a double, try again.");
            return "failed";
        }

        return s;
    }

    public static ChassisSpeeds calc(ChassisSpeeds setpoint) {
        var setpointTwist = new Pose2d()
                .log(
                    new Pose2d(
                        setpoint.vxMetersPerSecond * loopPeriodSecs,
                        setpoint.vyMetersPerSecond * loopPeriodSecs,
                        new Rotation2d(setpoint.omegaRadiansPerSecond * loopPeriodSecs)));
        var adjustedSpeeds = new ChassisSpeeds(
            setpointTwist.dx / loopPeriodSecs,
            setpointTwist.dy / loopPeriodSecs,
            setpointTwist.dtheta / loopPeriodSecs);

        return adjustedSpeeds;

        // SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(adjustedSpeeds);
        // SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearSpeed.get());
    }
}
