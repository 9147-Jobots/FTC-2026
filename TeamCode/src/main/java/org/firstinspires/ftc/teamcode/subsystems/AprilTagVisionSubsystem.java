package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Subsystem responsible for AprilTag detection and 3D pose estimation (X, Y, Z, Range, Bearing, Yaw).
 */
@SuppressWarnings("unused")
public class AprilTagVisionSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    public AprilTagVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super();

        this.telemetry = telemetry;

        // Create the AprilTag processor with default settings
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        // Create the VisionPortal with Camera1
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Camera1"),
                aprilTagProcessor
        );
    }

    /**
     * Returns the list of all currently detected AprilTags.
     */
    public List<AprilTagDetection> getDetections() {
        if (aprilTagProcessor == null) return new ArrayList<>();
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections != null ? detections : new ArrayList<>();
    }

    /**
     * Checks if any AprilTags are currently detected.
     */
    public boolean hasDetections() {
        return !getDetections().isEmpty();
    }

    /**
     * Returns the detection matching the specified AprilTag ID, or null if not detected.
     */
    public AprilTagDetection getDetectionById(int targetId) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }

    /**
     * Returns the detected AprilTag with the smallest 3D range (closest to the camera), or null if none detected.
     */
    public AprilTagDetection getClosestDetection() {
        AprilTagDetection closest = null;
        double minRange = Double.MAX_VALUE;

        for (AprilTagDetection detection : getDetections()) {
            if (detection.ftcPose != null && detection.ftcPose.range < minRange) {
                minRange = detection.ftcPose.range;
                closest = detection;
            }
        }
        return closest;
    }

    @Override
    public void periodic() {
        List<AprilTagDetection> detections = getDetections();
        telemetry.addData("AprilTag Count", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose != null) {
                String tagLabel = (detection.metadata != null) ? detection.metadata.name : "Tag " + detection.id;
                telemetry.addLine(String.format(Locale.US, "[%d] %s", detection.id, tagLabel));
                telemetry.addData("  X / Y / Z (in)", "%4.1f, %4.1f, %4.1f",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                telemetry.addData("  Range / Bearing", "%4.1fin @ %4.1f°",
                        detection.ftcPose.range, detection.ftcPose.bearing);
                telemetry.addData("  Yaw Rotation", "%4.1f°", detection.ftcPose.yaw);
            }
        }
    }

    /**
     * Safely closes the VisionPortal stream when OpMode terminates.
     */
    public void stopVision() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
