package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;
import org.firstinspires.ftc.teamcode.vision.AprilTagTargetGroup;
import org.firstinspires.ftc.teamcode.vision.VisionTargetResult;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Subsystem responsible for AprilTag detection, tag group filtering (TAG_36h11),
 * and 3D projected center estimation (X, Y, Z, Distance, Yaw).
 */
@SuppressWarnings("unused")
public class AprilTagVisionSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private AprilTagTargetGroup.Alliance activeAlliance = AprilTagTargetGroup.Alliance.BLUE;
    private AprilTagTargetGroup activeTargetGroup = null; // null means any group matching active alliance

    private VisionTargetResult latestProjectedResult = VisionTargetResult.noTarget();

    public AprilTagVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "Camera1");
    }

    public AprilTagVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String webcamName) {
        super();

        this.telemetry = telemetry;

        // Configure AprilTagProcessor with explicit TAG_36h11 family (standard FTC season tags)
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(900.0, 900.0, 640.0, 400.0)
                .build();

        // Use VisionPortal.Builder to configure 1280x800 high-resolution MJPEG stream format
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, webcamName));
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
     * Returns the latest projected center target result (distance in meters, yaw in degrees).
     */
    public VisionTargetResult getProjectedTargetResult() {
        return latestProjectedResult;
    }

    public void setActiveAlliance(AprilTagTargetGroup.Alliance alliance) {
        this.activeAlliance = alliance;
    }

    public AprilTagTargetGroup.Alliance getActiveAlliance() {
        return activeAlliance;
    }

    public void setActiveTargetGroup(AprilTagTargetGroup targetGroup) {
        this.activeTargetGroup = targetGroup;
    }

    public AprilTagTargetGroup getActiveTargetGroup() {
        return activeTargetGroup;
    }

    @Override
    public void periodic() {
        List<AprilTagDetection> detections = getDetections();
        telemetry.addData("AprilTag Count", detections.size());

        double totalCenterXInches = 0.0;
        double totalCenterYInches = 0.0;
        double totalCenterZInches = 0.0;
        int validDetectionCount = 0;
        int primaryTagId = -1;
        AprilTagTargetGroup primaryGroup = null;

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose == null) continue;

            int tagId = detection.id;
            AprilTagTargetGroup tagGroup = AprilTagTargetGroup.getGroupForTag(tagId);
            if (tagGroup == null) continue;

            // Filter by active group or active alliance
            boolean matchesGroup = (activeTargetGroup != null && tagGroup == activeTargetGroup);
            boolean matchesAlliance = (activeTargetGroup == null && tagGroup.getAlliance() == activeAlliance);

            if (matchesGroup || matchesAlliance) {
                double offsetInches = tagGroup.getTagHorizontalOffsetInches(tagId);
                double centerXInches = detection.ftcPose.x - offsetInches;
                double centerYInches = detection.ftcPose.y;
                double centerZInches = detection.ftcPose.z;

                totalCenterXInches += centerXInches;
                totalCenterYInches += centerYInches;
                totalCenterZInches += centerZInches;
                validDetectionCount++;

                if (primaryTagId == -1) {
                    primaryTagId = tagId;
                    primaryGroup = tagGroup;
                }
            }
        }

        if (validDetectionCount > 0) {
            double avgCenterXInches = totalCenterXInches / validDetectionCount;
            double avgCenterYInches = totalCenterYInches / validDetectionCount;
            double avgCenterZInches = totalCenterZInches / validDetectionCount;

            // Compute distance to projected center in meters
            double distInches = Math.sqrt(avgCenterXInches * avgCenterXInches + avgCenterYInches * avgCenterYInches + avgCenterZInches * avgCenterZInches);
            double distMeters = distInches * 0.0254;

            // Compute horizontal yaw offset to projected center in degrees
            double yawDegrees = Math.toDegrees(Math.atan2(avgCenterXInches, avgCenterYInches));

            latestProjectedResult = new VisionTargetResult(true, primaryTagId, primaryGroup, distMeters, yawDegrees);

            telemetry.addData("Target Center Dist (M)", "%4.2f", distMeters);
            telemetry.addData("Target Center Yaw (Deg)", "%4.1f°", yawDegrees);
            telemetry.addData("Valid Tags Count", validDetectionCount);
        } else {
            latestProjectedResult = VisionTargetResult.noTarget();
            telemetry.addData("Target Center Status", "No Target in Active Group/Alliance");
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
