package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;

/**
 * Selective system decider between Vision and Odometry target tracking applications.
 */
public class AimSourceSelector {

    /**
     * Enum return value representing the active decider choice between Vision and Odometry.
     */
    public enum AimSource {
        VISION,
        ODOMETRY;

        /**
         * Simple boolean check to decide if Vision is selected.
         */
        public boolean isVision() {
            return this == VISION;
        }

        /**
         * Simple boolean check to decide if Odometry is selected.
         */
        public boolean isOdometry() {
            return this == ODOMETRY;
        }
    }

    /**
     * Decides between using VISION and ODOMETRY positional values and lookup tables.
     * Rule: If valid alliance AprilTags are detected and projected by vision, use VISION; otherwise, use ODOMETRY.
     *
     * @param visionSubsystem the AprilTag vision subsystem to inspect for active target detections
     * @return AimSource.VISION if valid target is detected, AimSource.ODOMETRY otherwise.
     */
    public static AimSource selectAimSource(AprilTagVisionSubsystem visionSubsystem) {
        if (visionSubsystem != null && visionSubsystem.getProjectedTargetResult().isTargetDetected()) {
            return AimSource.VISION;
        }
        return AimSource.ODOMETRY;
    }

    /**
     * Overload for deciding aim source based on a direct boolean check of AprilTag visibility.
     *
     * @param hasAprilTags true if AprilTags are visible to vision
     * @return AimSource.VISION if true, AimSource.ODOMETRY otherwise.
     */
    public static AimSource selectAimSource(boolean hasAprilTags) {
        return hasAprilTags ? AimSource.VISION : AimSource.ODOMETRY;
    }
}
