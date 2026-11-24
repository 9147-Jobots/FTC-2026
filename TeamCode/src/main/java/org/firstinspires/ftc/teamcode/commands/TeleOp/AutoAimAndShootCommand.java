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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
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
 * Command responsible for automatic vision and odometry aiming and 2D-velocity-compensated shooter control:
 * - Uses AimSourceSelector to decide between Vision and Odometry aiming sources.
 * - Computes projected target distance & yaw offset.
 * - Manages a 3-State Machine: RAMPING_UP, ACTIVE_SHOOTING, and PAUSED_RECOVERING.
 * - RAMPING_UP: Holds hood at nominal angle until flywheel velocity & heading targets are ready.
 * - ACTIVE_SHOOTING: Uses 2D bilinear interpolation to continuously adjust hood angle for real-time measured velocity sags.
 * - PAUSED_RECOVERING: If flywheel velocity sags below hard minimum threshold, feeding pauses until velocity recovers.
 */
@SuppressWarnings("unused")
public class AutoAimAndShootCommand implements Command {

    public enum ShooterState {
        RAMPING_UP,        // Flywheel spinning up; hood held at nominal angle; feeding locked
        ACTIVE_SHOOTING,   // Active feeding; hood continuously adjusts via 2D bilinear lookup against actual velocity
        PAUSED_RECOVERING  // Severe RPM sag (< 1150 TPS); feeding paused until flywheel recovers
    }

    private final ShooterSubsystem shooterSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final AprilTagVisionSubsystem visionSubsystem;
    private final Odometry odometry;
    private final Gamepad gamepad;

    private final DoubleSupplier distanceSupplierMeters;
    private final DoubleSupplier yawOffsetDegreesSupplier;
    private final BooleanSupplier hasTargetSupplier;
    private final ShooterLookupTable lookupTable;

    private final ElapsedTime commandTimer = new ElapsedTime();
    private static final double OVERRIDE_TIMEOUT_SECONDS = 3.0;
    private static final double MIN_OPERABLE_VELOCITY_TPS = 1150.0; // Hard minimum threshold before pausing feeder

    public static final double DEFAULT_FALLBACK_DISTANCE_METERS = 1.2;

    private static final double KP_YAW = 0.015;
    private static final double MAX_YAW_POWER = 0.30; // 30% maximum turning power cap
    private static final double HEADING_TOLERANCE_DEGREES = 3.0;

    private boolean autoFeedingActive = false;
    private AimSource lastAimSource = AimSource.ODOMETRY;
    private ShooterState currentState = ShooterState.RAMPING_UP;

    /** Primary Constructor accepting all required subsystems and parameters. */
    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  IndexerSubsystem indexerSubsystem,
                                  IntakeSubsystem intakeSubsystem,
                                  AprilTagVisionSubsystem visionSubsystem,
                                  Odometry odometry,
                                  Gamepad gamepad,
                                  ShooterLookupTable lookupTable) {
        this.shooterSubsystem = shooterSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
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

    @Override
    public Set<Subsystem> getRequirements() {
        return new HashSet<>(Arrays.asList(shooterSubsystem, driveSubsystem, indexerSubsystem));
    }

    @Override
    public void init() {
        commandTimer.reset();
        autoFeedingActive = false;
        currentState = ShooterState.RAMPING_UP;
    }

    @Override
    public void execute() {
        // Select between Vision and Odometry sources using AimSourceSelector method
        AimSource source = visionSubsystem != null ?
                AimSourceSelector.selectAimSource(visionSubsystem) :
                AimSourceSelector.selectAimSource(hasTargetSupplier.getAsBoolean());
        this.lastAimSource = source;

        double distanceMeters;
        double yawErrorDegrees;
        boolean hasTarget;

        if (source.isVision()) {
            hasTarget = hasTargetSupplier.getAsBoolean();
            distanceMeters = hasTarget ? distanceSupplierMeters.getAsDouble() : DEFAULT_FALLBACK_DISTANCE_METERS;
            yawErrorDegrees = hasTarget ? yawOffsetDegreesSupplier.getAsDouble() : 0.0;
        } else {
            if (odometry != null && odometry.getPose() != null) {
                Pose2D pose = odometry.getPose();
                double robotX = pose.getX(DistanceUnit.METER);
                double robotY = pose.getY(DistanceUnit.METER);

                double[] hivePos = getTargetHivePosition(robotX, robotY);
                double targetHiveX = hivePos[0];
                double targetHiveY = hivePos[1];

                double dx = targetHiveX - robotX;
                double dy = targetHiveY - robotY;

                distanceMeters = Math.hypot(dx, dy);

                double targetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
                double currentHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

                double headingErrorDeg = AngleUnit.normalizeDegrees(targetAngleDeg - currentHeadingDeg);
                yawErrorDegrees = -headingErrorDeg;
                hasTarget = true;
            } else {
                distanceMeters = DEFAULT_FALLBACK_DISTANCE_METERS;
                yawErrorDegrees = 0.0;
                hasTarget = false;
            }
        }

        // Read real-time low-pass filtered flywheel velocity from subsystem
        double actualVelocity = shooterSubsystem.getFlywheelVelocity();

        // Query 2D Lookup Table based on state
        ShooterLookupTable.ShooterTargetState targetState;
        if (currentState == ShooterState.ACTIVE_SHOOTING) {
            // ACTIVE SHOOTING: Query 2D Bilinear map using real-time measured velocity
            targetState = lookupTable.get2DInterpolatedState(distanceMeters, actualVelocity, source);
        } else {
            // RAMPING / RECOVERING: Query nominal target state
            targetState = lookupTable.getInterpolatedState(distanceMeters, source);
        }

        // Apply flywheel target velocity & hood angle
        shooterSubsystem.setTargetState(targetState);

        // Compute drivetrain heading turning power (capped at 30%)
        double yawPower = hasTarget ? Math.max(-MAX_YAW_POWER, Math.min(MAX_YAW_POWER, KP_YAW * yawErrorDegrees)) : 0.0;

        // Combine driver translation inputs with automatic heading yaw
        double axial = gamepad != null ? -gamepad.left_stick_y : 0.0;
        double lateral = gamepad != null ? gamepad.left_stick_x : 0.0;

        driveSubsystem.drive(axial, lateral, yawPower);

        // State Machine Readiness Checks
        boolean timeoutOverride = commandTimer.seconds() >= OVERRIDE_TIMEOUT_SECONDS;
        boolean velocityOnTarget = shooterSubsystem.isReady();
        boolean headingOnTarget = !hasTarget || (Math.abs(yawErrorDegrees) <= HEADING_TOLERANCE_DEGREES);

        switch (currentState) {
            case RAMPING_UP:
                if ((velocityOnTarget && headingOnTarget) || timeoutOverride) {
                    currentState = ShooterState.ACTIVE_SHOOTING;
                    startFeeding();
                }
                break;

            case ACTIVE_SHOOTING:
                if (actualVelocity < MIN_OPERABLE_VELOCITY_TPS) {
                    stopFeeding();
                    currentState = ShooterState.PAUSED_RECOVERING;
                }
                break;

            case PAUSED_RECOVERING:
                if (velocityOnTarget && headingOnTarget) {
                    currentState = ShooterState.ACTIVE_SHOOTING;
                    startFeeding();
                }
                break;
        }
    }

    private void startFeeding() {
        shooterSubsystem.startKickerPulsing();
        indexerSubsystem.runIndexer(1.0);
        intakeSubsystem.runIntake();
        autoFeedingActive = true;
    }

    private void stopFeeding() {
        shooterSubsystem.stopKicker();
        indexerSubsystem.stopIndexer();
        intakeSubsystem.stopIntake();
        autoFeedingActive = false;
    }

    /** Resolves target Hive center (X, Y in meters). */
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

    public AimSource getActiveAimSource() {
        return lastAimSource;
    }

    public ShooterState getShooterState() {
        return currentState;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
        stopFeeding();
        driveSubsystem.drive(0, 0, 0);
        currentState = ShooterState.RAMPING_UP;
    }
}
