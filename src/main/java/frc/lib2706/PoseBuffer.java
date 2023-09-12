// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2706;

import java.util.NoSuchElementException;
import java.util.Optional;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;

/** Add your docs here. */
public class PoseBuffer {
    private final double BUFFER_DURATION = 1.5;

    private final TimeInterpolatableBuffer<Pose2d> m_poseBuffer = TimeInterpolatableBuffer
            .createBuffer(BUFFER_DURATION);

    public void addPoseToBuffer(Pose2d pose) {
        m_poseBuffer.addSample(
            MathSharedStore.getTimestamp(),
            pose);
    }

    public Optional<Pose2d> getPoseAtTimestamp(double timestampSeconds) {
        // If this measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (m_poseBuffer.getInternalBuffer().lastKey() - BUFFER_DURATION > timestampSeconds) {
                return Optional.empty();
            }
        } catch (NoSuchElementException ex) {
            return Optional.empty();
        }

        // Get the pose odometry measured at the moment the vision measurement was made.
        return m_poseBuffer.getSample(timestampSeconds);
    }
}
