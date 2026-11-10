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
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Subsystem responsible for AprilTag detection, tag group filtering (TAG_36h11),
 * 50° camera pitch 3D rotation, and floor-projected center target estimation.
 */
@SuppressWarnings({"unused", "ReassignedVariable"})
public class AprilTagVisionSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private AprilTagTargetGroup.Alliance activeAlliance = AprilTagTargetGroup.Alliance.BLUE;
    private AprilTagTargetGroup activeTargetGroup = null; // null means any group matching active alliance

    // Camera mount pitch orientation angle (50.0 degrees tilted upwards from horizontal floor)
    public static double CAMERA_PITCH_DEGREES = 50.0;

    private VisionTargetResult latestProjectedResult = VisionTargetResult.noTarget();

    public AprilTagVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, "Camera1");
    }

    public AprilTagVisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String webcamName) {
        super();

        this.telemetry = telemetry;

        // Build Custom AprilTag Library registering team tags 30-45 with exact 3.25 inch tag size
        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();
        for (int tagId = 30; tagId <= 45; tagId++) {
            libraryBuilder.addTag(new AprilTagMetadata(
                    tagId,
                    "Tag " + tagId,
                    3.25, // Tag size: 3.25 inches
                    DistanceUnit.INCH
            ));
        }
        AprilTagLibrary teamTagLibrary = libraryBuilder.build();

        // Configure AprilTagProcessor with explicit TAG_36h11 family, custom TagLibrary, and Arducam intrinsics
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(teamTagLibrary)
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
        telemetry.addData("Active Alliance", activeAlliance.name());

        double pitchRad = Math.toRadians(CAMERA_PITCH_DEGREES);
        double cosPitch = Math.cos(pitchRad);
        double sinPitch = Math.sin(pitchRad);

        // Find the closest group matching active alliance (or use explicitly activeTargetGroup)
        AprilTagTargetGroup selectedGroup = activeTargetGroup;
        if (selectedGroup == null) {
            double minGroupRange = Double.MAX_VALUE;
            for (AprilTagDetection detection : detections) {
                String poseStatus = (detection.ftcPose != null) ?
                        String.format(Locale.US, "Range %.1fin, Yaw %.1f°", detection.ftcPose.range, detection.ftcPose.yaw) : "No 3D Pose";
                telemetry.addLine(String.format(Locale.US, "Raw Tag #%d: %s", detection.id, poseStatus));

                if (detection.ftcPose == null) continue;

                int tagId = detection.id;
                AprilTagTargetGroup tagGroup = AprilTagTargetGroup.getGroupForTag(tagId);
                if (tagGroup == null) continue;

                if (tagGroup.getAlliance() == activeAlliance) {
                    if (detection.ftcPose.range < minGroupRange) {
                        minGroupRange = detection.ftcPose.range;
                        selectedGroup = tagGroup;
                    }
                }
            }
        }

        double totalCenterXInches = 0.0;
        double totalCenterYInches = 0.0;
        int validDetectionCount = 0;
        int primaryTagId = -1;

        // Process detections belonging ONLY to the closest selected group
        if (selectedGroup != null) {
            for (AprilTagDetection detection : detections) {
                if (detection.ftcPose == null) continue;

                int tagId = detection.id;
                AprilTagTargetGroup tagGroup = AprilTagTargetGroup.getGroupForTag(tagId);
                if (tagGroup == selectedGroup) {
                    double xCam = detection.ftcPose.x;
                    double yCam = detection.ftcPose.y;
                    double zCam = detection.ftcPose.z;

                    // Rotate 3D pose from 50° tilted camera frame to horizontal floor frame
                    double yFloor = yCam * cosPitch - zCam * sinPitch;

                    // Apply group horizontal offset to projected target center
                    double offsetInches = tagGroup.getTagHorizontalOffsetInches(tagId);
                    double centerXFloorInches = xCam - offsetInches;

                    totalCenterXInches += centerXFloorInches;
                    totalCenterYInches += yFloor;
                    validDetectionCount++;

                    if (primaryTagId == -1) {
                        primaryTagId = tagId;
                    }
                }
            }
        }

        if (validDetectionCount > 0) {
            double avgCenterXInches = totalCenterXInches / validDetectionCount;
            double avgCenterYInches = totalCenterYInches / validDetectionCount;

            // Compute true 2D projected horizontal distance on floor (in meters)
            double dist2DFloorInches = Math.hypot(avgCenterXInches, avgCenterYInches);
            double distMeters = dist2DFloorInches * 0.0254;

            // Compute true horizontal yaw offset to projected center in floor plane (in degrees)
            double yawDegrees = Math.toDegrees(Math.atan2(avgCenterXInches, avgCenterYInches));

            latestProjectedResult = new VisionTargetResult(true, primaryTagId, selectedGroup, distMeters, yawDegrees);

            telemetry.addData("Selected Closest Group", selectedGroup.name());
            telemetry.addData("Floor Center Dist (M / Inches)", "%4.2fm (%4.1fin)", distMeters, dist2DFloorInches);
            telemetry.addData("Floor Center X / Y (Inches)", "X: %4.1fin, Y: %4.1fin", avgCenterXInches, avgCenterYInches);
            telemetry.addData("Floor Center Yaw (Deg)", "%4.1f°", yawDegrees);
            telemetry.addData("Group Valid Tags Count", validDetectionCount);
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
