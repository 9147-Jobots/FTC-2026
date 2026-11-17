package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.AimSourceSelector;
import org.firstinspires.ftc.teamcode.util.AimSourceSelector.AimSource;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.vision.AprilTagTargetGroup;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command responsible for automatic vision and odometry aiming and shooter control:
 * - Uses AimSourceSelector to decide between Vision and Odometry aiming sources.
 * - If Vision is selected: reads projected target distance & yaw offset from AprilTagVisionSubsystem and interpolates via vision lookup table.
 * - If Odometry is selected: calculates 2D distance between odometry pose and absolute Hive center (Constants.HIVE_CENTER_X_METERS, HIVE_CENTER_Y_METERS) and interpolates via 8-point odometry array.
 * - Applies proportional turning power (capped at 20% max speed) to DriveSubsystem based on yaw offset.
 * - Automatically activates Kicker (100%) and Indexer (100%) when flywheel speed/heading are ready OR after 3 seconds held.
 */
@SuppressWarnings("unused")
public class AutoAimAndShootCommand implements Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final AprilTagVisionSubsystem visionSubsystem;
    private final Odometry odometry;
    private final Gamepad gamepad;

    private final DoubleSupplier distanceSupplierMeters;
    private final DoubleSupplier yawOffsetDegreesSupplier;
    private final BooleanSupplier hasTargetSupplier;
    private final ShooterLookupTable lookupTable;

    private final ElapsedTime commandTimer = new ElapsedTime();
    private static final double OVERRIDE_TIMEOUT_SECONDS = 3.0;

    public static final double DEFAULT_FALLBACK_DISTANCE_METERS = 1.2;

    private static final double KP_YAW = 0.015; // 20% max power at 30 degrees error (0.20 / 30.0)
    private static final double MAX_YAW_POWER = 0.20; // 20% maximum turning power cap
    private static final double VELOCITY_TOLERANCE_TPS = 100.0;
    private static final double HEADING_TOLERANCE_DEGREES = 3.0;

    private boolean autoFeedingActive = false;

    /** Constructor accepting both AprilTagVisionSubsystem and Odometry. */
    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  IndexerSubsystem indexerSubsystem,
                                  AprilTagVisionSubsystem visionSubsystem,
                                  Odometry odometry,
                                  Gamepad gamepad,
                                  ShooterLookupTable lookupTable) {
        this.shooterSubsystem = shooterSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.odometry = odometry;
        this.gamepad = gamepad;
        this.lookupTable = lookupTable;

        this.distanceSupplierMeters = () -> (visionSubsystem != null && visionSubsystem.getProjectedTargetResult().isTargetDetected()) ?
                visionSubsystem.getProjectedTargetResult().getDistanceMeters() : DEFAULT_FALLBACK_DISTANCE_METERS;
        this.yawOffsetDegreesSupplier = () -> (visionSubsystem != null && visionSubsystem.getProjectedTargetResult().isTargetDetected()) ?
                visionSubsystem.getProjectedTargetResult().getYawDegrees() : 0.0;
        this.hasTargetSupplier = () -> visionSubsystem != null && visionSubsystem.getProjectedTargetResult().isTargetDetected();
    }

    /** Constructor for direct VisionSubsystem AprilTag input without Odometry subsystem. */
    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  IndexerSubsystem indexerSubsystem,
                                  AprilTagVisionSubsystem visionSubsystem,
                                  Gamepad gamepad,
                                  ShooterLookupTable lookupTable) {
        this(shooterSubsystem, driveSubsystem, indexerSubsystem, visionSubsystem, null, gamepad, lookupTable);
    }

    /** Constructor for direct Camera / Vision suppliers input with Odometry subsystem. */
    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  IndexerSubsystem indexerSubsystem,
                                  Gamepad gamepad,
                                  DoubleSupplier distanceSupplierMeters,
                                  DoubleSupplier yawOffsetDegreesSupplier,
                                  BooleanSupplier hasTargetSupplier,
                                  Odometry odometry,
                                  ShooterLookupTable lookupTable) {
        this.shooterSubsystem = shooterSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.visionSubsystem = null;
        this.odometry = odometry;
        this.gamepad = gamepad;
        this.distanceSupplierMeters = distanceSupplierMeters;
        this.yawOffsetDegreesSupplier = yawOffsetDegreesSupplier;
        this.hasTargetSupplier = hasTargetSupplier;
        this.lookupTable = lookupTable;
    }

    /** Constructor for direct Camera / Vision suppliers input (Limelight, AprilTag, OpenCV, HuskyLens). */
    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  IndexerSubsystem indexerSubsystem,
                                  Gamepad gamepad,
                                  DoubleSupplier distanceSupplierMeters,
                                  DoubleSupplier yawOffsetDegreesSupplier,
                                  BooleanSupplier hasTargetSupplier,
                                  ShooterLookupTable lookupTable) {
        this(shooterSubsystem, driveSubsystem, indexerSubsystem, gamepad, distanceSupplierMeters, yawOffsetDegreesSupplier, hasTargetSupplier, null, lookupTable);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return new HashSet<>(Arrays.asList(shooterSubsystem, driveSubsystem, indexerSubsystem));
    }

    @Override
    public void init() {
        commandTimer.reset();
    }

    @Override
    public void execute() {
        // Select between Vision and Odometry sources using AimSourceSelector method
        AimSource source = visionSubsystem != null ?
                AimSourceSelector.selectAimSource(visionSubsystem) :
                AimSourceSelector.selectAimSource(hasTargetSupplier.getAsBoolean());

        double distanceMeters;
        double yawErrorDegrees;
        ShooterLookupTable.ShooterTargetState targetState;
        boolean hasTarget;

        // Perform simple boolean check on AimSource enum return value
        if (source.isVision()) {
            // VISION selected: implement existing camera vision system and apply to vision lookup table
            hasTarget = hasTargetSupplier.getAsBoolean();
            distanceMeters = hasTarget ? distanceSupplierMeters.getAsDouble() : DEFAULT_FALLBACK_DISTANCE_METERS;
            yawErrorDegrees = hasTarget ? yawOffsetDegreesSupplier.getAsDouble() : 0.0;

            targetState = lookupTable.getVisionInterpolatedState(distanceMeters);
        } else {
            // ODOMETRY selected: use odometry field-position in combination with absolute centre-of-hive position values
            if (odometry != null && odometry.getPose() != null) {
                Pose2D pose = odometry.getPose();
                double robotX = pose.getX(DistanceUnit.METER);
                double robotY = pose.getY(DistanceUnit.METER);

                // Determine target Hive position among 4 hives across Blue and Red alliances
                double[] hivePos = getTargetHivePosition(robotX, robotY);
                double targetHiveX = hivePos[0];
                double targetHiveY = hivePos[1];

                double dx = targetHiveX - robotX;
                double dy = targetHiveY - robotY;

                // Calculate horizontal distance between robot and selected centre of hive
                distanceMeters = Math.hypot(dx, dy);

                // Compute yaw heading error relative to target hive position
                double targetAngleDeg = Math.toDegrees(Math.atan2(dx, dy));
                double currentHeadingDeg = pose.getHeading(AngleUnit.DEGREES);
                yawErrorDegrees = normalizeAngleDeg(targetAngleDeg - currentHeadingDeg);
                hasTarget = true;
            } else {
                distanceMeters = DEFAULT_FALLBACK_DISTANCE_METERS;
                yawErrorDegrees = 0.0;
                hasTarget = false;
            }

            // Apply calculated distance to the 8-point odometry array to return corresponding rpm and hood angle
            targetState = lookupTable.getOdometryInterpolatedState(distanceMeters);
        }

        // Apply calculated flywheel velocity (rpm) and hood angle state to shooter subsystem
        shooterSubsystem.setTargetState(targetState);

        // Compute proportional heading yaw correction for drivetrain (capped at 20% max power)
        double yawPower = hasTarget ? Math.max(-MAX_YAW_POWER, Math.min(MAX_YAW_POWER, KP_YAW * yawErrorDegrees)) : 0.0;

        // Combine driver translation inputs (left stick) with automatic heading yaw
        double axial = gamepad != null ? -gamepad.left_stick_y : 0.0;
        double lateral = gamepad != null ? gamepad.left_stick_x : 0.0;

        driveSubsystem.drive(axial, lateral, yawPower);

        // Check readiness: velocity + heading tolerances OR 3-second held override timeout
        boolean timeoutOverride = commandTimer.seconds() >= OVERRIDE_TIMEOUT_SECONDS;
        boolean velocityOnTarget = Math.abs(shooterSubsystem.getFlywheelVelocity() - targetState.getFlywheelVelocity()) <= VELOCITY_TOLERANCE_TPS;
        boolean headingOnTarget = !hasTarget || (Math.abs(yawErrorDegrees) <= HEADING_TOLERANCE_DEGREES);

        boolean readyToFeed = timeoutOverride || (velocityOnTarget && headingOnTarget);

        if (readyToFeed) {
            shooterSubsystem.runKickerPower(1.0);
            indexerSubsystem.startPulsing();
            autoFeedingActive = true;
        } else if (autoFeedingActive) {
            shooterSubsystem.stopKicker();
            indexerSubsystem.stopIndexer();
            autoFeedingActive = false;
        }
    }

    /**
     * Resolves target Hive center (X, Y in meters) among the 4 Hives across Blue and Red alliances.
     * Uses active alliance/target group from vision subsystem if available, or picks the closest alliance hive.
     */
    private double[] getTargetHivePosition(double robotX, double robotY) {
        AprilTagTargetGroup.Alliance alliance = (visionSubsystem != null) ? visionSubsystem.getActiveAlliance() : AprilTagTargetGroup.Alliance.BLUE;
        AprilTagTargetGroup targetGroup = (visionSubsystem != null) ? visionSubsystem.getActiveTargetGroup() : null;

        if (targetGroup != null) {
            switch (targetGroup) {
                case BLUE_GROUP_1:
                    return new double[]{Constants.BLUE_HIVE_1_CENTER_X_METERS, Constants.BLUE_HIVE_1_CENTER_Y_METERS};
                case BLUE_GROUP_2:
                    return new double[]{Constants.BLUE_HIVE_2_CENTER_X_METERS, Constants.BLUE_HIVE_2_CENTER_Y_METERS};
                case RED_GROUP_1:
                    return new double[]{Constants.RED_HIVE_1_CENTER_X_METERS, Constants.RED_HIVE_1_CENTER_Y_METERS};
                case RED_GROUP_2:
                    return new double[]{Constants.RED_HIVE_2_CENTER_X_METERS, Constants.RED_HIVE_2_CENTER_Y_METERS};
            }
        }

        // If specific target group is null, pick closest Hive within active alliance
        double[][] candidateHives;
        if (alliance == AprilTagTargetGroup.Alliance.RED) {
            candidateHives = new double[][]{
                    {Constants.RED_HIVE_1_CENTER_X_METERS, Constants.RED_HIVE_1_CENTER_Y_METERS},
                    {Constants.RED_HIVE_2_CENTER_X_METERS, Constants.RED_HIVE_2_CENTER_Y_METERS}
            };
        } else {
            candidateHives = new double[][]{
                    {Constants.BLUE_HIVE_1_CENTER_X_METERS, Constants.BLUE_HIVE_1_CENTER_Y_METERS},
                    {Constants.BLUE_HIVE_2_CENTER_X_METERS, Constants.BLUE_HIVE_2_CENTER_Y_METERS}
            };
        }

        double minDistanceSq = Double.MAX_VALUE;
        double[] closestHive = candidateHives[0];

        for (double[] hive : candidateHives) {
            double distSq = Math.pow(hive[0] - robotX, 2) + Math.pow(hive[1] - robotY, 2);
            if (distSq < minDistanceSq) {
                minDistanceSq = distSq;
                closestHive = hive;
            }
        }

        return closestHive;
    }

    private double normalizeAngleDeg(double angle) {
        double rem = angle % 360.0;
        if (rem > 180.0) {
            return rem - 360.0;
        } else if (rem < -180.0) {
            return rem + 360.0;
        }
        return rem;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        shooterSubsystem.stopKicker();
        indexerSubsystem.stopIndexer();
        driveSubsystem.drive(0, 0, 0);
        autoFeedingActive = false;
    }
}
