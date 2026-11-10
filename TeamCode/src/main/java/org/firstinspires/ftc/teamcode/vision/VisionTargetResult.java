package org.firstinspires.ftc.teamcode.vision;

import androidx.annotation.NonNull;
import java.util.Locale;

/**
 * Encapsulates AprilTag vision detection results projected to the target group center.
 */
@SuppressWarnings("unused")
public class VisionTargetResult {

    private final boolean targetDetected;
    private final int primaryTagId;
    private final AprilTagTargetGroup group;
    private final double distanceMeters;
    private final double yawDegrees;

    public VisionTargetResult(boolean targetDetected, int primaryTagId, AprilTagTargetGroup group, double distanceMeters, double yawDegrees) {
        this.targetDetected = targetDetected;
        this.primaryTagId = primaryTagId;
        this.group = group;
        this.distanceMeters = distanceMeters;
        this.yawDegrees = yawDegrees;
    }

    public static VisionTargetResult noTarget() {
        return new VisionTargetResult(false, -1, null, 0.0, 0.0);
    }

    public boolean isTargetDetected() {
        return targetDetected;
    }

    public int getPrimaryTagId() {
        return primaryTagId;
    }

    public AprilTagTargetGroup getGroup() {
        return group;
    }

    public double getDistanceMeters() {
        return distanceMeters;
    }

    public double getYawDegrees() {
        return yawDegrees;
    }

    @NonNull
    @Override
    public String toString() {
        if (!targetDetected) {
            return "No Target Detected";
        }
        return String.format(Locale.US, "Tag %d (%s) -> Center Dist: %.2fm, Yaw: %.1f°",
                primaryTagId, group != null ? group.name() : "Unknown", distanceMeters, yawDegrees);
    }
}
