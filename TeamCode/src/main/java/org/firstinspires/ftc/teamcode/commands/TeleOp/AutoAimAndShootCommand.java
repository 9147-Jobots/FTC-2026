package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command responsible for automatic vision aiming and shooter control:
 * - Reads projected target center distance and yaw offset from AprilTagVisionSubsystem or vision suppliers.
 * - Interpolates flywheel velocity (ticks/sec) and hood angle (degrees) via ShooterLookupTable.
 * - Applies proportional turning power (capped at 20% max speed) to DriveSubsystem based on yaw offset.
 * - Automatically activates Kicker (100%) and Indexer (100%) when flywheel speed and heading alignment are ready.
 */
@SuppressWarnings("unused")
public class AutoAimAndShootCommand implements Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final Gamepad gamepad;

    private final DoubleSupplier distanceSupplierMeters;
    private final DoubleSupplier yawOffsetDegreesSupplier;
    private final BooleanSupplier hasTargetSupplier;
    private final ShooterLookupTable lookupTable;

    private static final double KP_YAW = 0.00667; // 20% max power at 30 degrees error (0.20 / 30.0)
    private static final double MAX_YAW_POWER = 0.20; // 20% maximum turning power cap
    private static final double VELOCITY_TOLERANCE_TPS = 100.0;
    private static final double HEADING_TOLERANCE_DEGREES = 3.0;

    private boolean autoFeedingActive = false;

    /** Constructor for direct VisionSubsystem AprilTag input. */
    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  IndexerSubsystem indexerSubsystem,
                                  AprilTagVisionSubsystem visionSubsystem,
                                  Gamepad gamepad,
                                  ShooterLookupTable lookupTable) {
        this(
            shooterSubsystem,
            driveSubsystem,
            indexerSubsystem,
            gamepad,
            () -> visionSubsystem.getProjectedTargetResult().getDistanceMeters(),
            () -> visionSubsystem.getProjectedTargetResult().getYawDegrees(),
            () -> visionSubsystem.getProjectedTargetResult().isTargetDetected(),
            lookupTable
        );
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
        this.shooterSubsystem = shooterSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.gamepad = gamepad;
        this.distanceSupplierMeters = distanceSupplierMeters;
        this.yawOffsetDegreesSupplier = yawOffsetDegreesSupplier;
        this.hasTargetSupplier = hasTargetSupplier;
        this.lookupTable = lookupTable;
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return new HashSet<>(Arrays.asList(shooterSubsystem, driveSubsystem, indexerSubsystem));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        boolean hasTarget = hasTargetSupplier.getAsBoolean();

        double distanceMeters = distanceSupplierMeters.getAsDouble();
        double yawErrorDegrees = yawOffsetDegreesSupplier.getAsDouble();

        // Retrieve interpolated shooter parameters (flywheel velocity & hood angle)
        ShooterLookupTable.ShooterTargetState targetState = lookupTable.getInterpolatedState(distanceMeters);
        shooterSubsystem.setTargetState(targetState);

        // Compute proportional heading yaw correction for drivetrain (capped at 20% max power)
        double yawPower = hasTarget ? Math.max(-MAX_YAW_POWER, Math.min(MAX_YAW_POWER, KP_YAW * yawErrorDegrees)) : 0.0;

        // Combine driver translation inputs (left stick) with automatic heading yaw
        double axial = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;

        driveSubsystem.drive(axial, lateral, yawPower);

        // Check if flywheel velocity and heading alignment are ready for firing
        boolean velocityOnTarget = Math.abs(shooterSubsystem.getFlywheelVelocity() - targetState.getFlywheelVelocity()) <= VELOCITY_TOLERANCE_TPS;
        boolean headingOnTarget = hasTarget && Math.abs(yawErrorDegrees) <= HEADING_TOLERANCE_DEGREES;

        if (velocityOnTarget && headingOnTarget) {
            shooterSubsystem.runKickerPower(1.0);
            indexerSubsystem.runIndexer(1.0);
            autoFeedingActive = true;
        } else if (autoFeedingActive) {
            shooterSubsystem.stopKicker();
            indexerSubsystem.stopIndexer();
            autoFeedingActive = false;
        }
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
