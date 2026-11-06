package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

        // Configure AprilTagProcessor with explicit Inch & Degree units and 1280x800 Arducam calibration intrinsics
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(900.0, 900.0, 640.0, 400.0)
                .build();

        // Use VisionPortal.Builder to configure 1280x800 high-resolution MJPEG stream format
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Camera1"));
        builder.addProcessor(aprilTagProcessor);
        builder.setCameraResolution(new Size(1280, 800));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.enableLiveView(true);

        visionPortal = builder.build();
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
            String tagLabel = (detection.metadata != null) ? detection.metadata.name : "Tag " + detection.id;
            if (detection.ftcPose != null) {
                telemetry.addLine(String.format(Locale.US, "[%d] %s", detection.id, tagLabel));
                telemetry.addData("  X / Y / Z (in)", "%4.1f, %4.1f, %4.1f",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                telemetry.addData("  Range / Bearing", "%4.1fin @ %4.1f°",
                        detection.ftcPose.range, detection.ftcPose.bearing);
                telemetry.addData("  Yaw Rotation", "%4.1f°", detection.ftcPose.yaw);
            } else {
                telemetry.addLine(String.format(Locale.US, "[%d] %s (No 3D Pose)", detection.id, tagLabel));
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
