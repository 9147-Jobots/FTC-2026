package org.firstinspires.ftc.teamcode.commands.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command responsible for automatic aiming and shooter control in TeleOp/Auto:
 * - Supports direct camera vision input (distance and horizontal yaw offset) or Odometry calculation.
 * - Interpolates flywheel velocity (ticks/sec) and hood angle (degrees) via ShooterLookupTable.
 * - Applies proportional turning power to DriveSubsystem based on camera/odometry yaw offset.
 * - Runs kicker motor when driver pulls trigger and velocity/heading tolerances are met.
 */
@SuppressWarnings("unused")
public class AutoAimAndShootCommand implements Command {

    private final ShooterSubsystem shooterSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final Gamepad gamepad;

    private final DoubleSupplier distanceSupplierMeters;
    private final DoubleSupplier yawOffsetDegreesSupplier;
    private final BooleanSupplier hasTargetSupplier;
    private final ShooterLookupTable lookupTable;

    private static final double KP_YAW = 0.035; // Proportional gain for yaw error in degrees
    private static final double HEADING_TOLERANCE_DEGREES = 3.0;
    private static final double VELOCITY_TOLERANCE_TPS = 100.0;

    private boolean kickerRunning = false;

    /** Constructor for direct Camera / Vision input (Limelight, AprilTag, OpenCV, HuskyLens). */
    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  Gamepad gamepad,
                                  DoubleSupplier distanceSupplierMeters,
                                  DoubleSupplier yawOffsetDegreesSupplier,
                                  BooleanSupplier hasTargetSupplier,
                                  ShooterLookupTable lookupTable) {
        this.shooterSubsystem = shooterSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.gamepad = gamepad;
        this.distanceSupplierMeters = distanceSupplierMeters;
        this.yawOffsetDegreesSupplier = yawOffsetDegreesSupplier;
        this.hasTargetSupplier = hasTargetSupplier;
        this.lookupTable = lookupTable;
    }

    /** Constructor for Odometry field-coordinate input. */
    public AutoAimAndShootCommand(ShooterSubsystem shooterSubsystem,
                                  DriveSubsystem driveSubsystem,
                                  Odometry odometry,
                                  Gamepad gamepad,
                                  double targetGoalX,
                                  double targetGoalY,
                                  ShooterLookupTable lookupTable) {
        this(
            shooterSubsystem,
            driveSubsystem,
            gamepad,
            () -> {
                Pose2D pose = odometry.getPose();
                double dx = targetGoalX - pose.getX(DistanceUnit.METER);
                double dy = targetGoalY - pose.getY(DistanceUnit.METER);
                return Math.hypot(dx, dy);
            },
            () -> {
                Pose2D pose = odometry.getPose();
                double dx = targetGoalX - pose.getX(DistanceUnit.METER);
                double dy = targetGoalY - pose.getY(DistanceUnit.METER);
                double targetHeadingRad = Math.atan2(dy, dx);
                double headingRad = pose.getHeading(AngleUnit.RADIANS);
                return Math.toDegrees(normalizeAngleRadians(targetHeadingRad - headingRad));
            },
            () -> true,
            lookupTable
        );
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return new HashSet<>(Arrays.asList(shooterSubsystem, driveSubsystem));
    }

    @Override
    public void init() {}

    @Override
    public void execute() {
        boolean hasTarget = hasTargetSupplier.getAsBoolean();

        double distanceMeters = distanceSupplierMeters.getAsDouble();
        double yawErrorDegrees = yawOffsetDegreesSupplier.getAsDouble();

        // Retrieve interpolated shooter parameters
        ShooterLookupTable.ShooterTargetState targetState = lookupTable.getInterpolatedState(distanceMeters);
        shooterSubsystem.setTargetState(targetState);

        // Compute proportional heading yaw correction
        double yawPower = hasTarget ? Math.max(-1.0, Math.min(1.0, KP_YAW * yawErrorDegrees)) : 0.0;

        // Combine driver translation inputs with automatic heading yaw
        double axial = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;

        driveSubsystem.drive(axial, lateral, yawPower);

        // Check velocity and heading alignment tolerance
        boolean velocityOnTarget = Math.abs(shooterSubsystem.getFlywheelVelocity() - targetState.getFlywheelVelocity()) <= VELOCITY_TOLERANCE_TPS;
        boolean headingOnTarget = hasTarget && Math.abs(yawErrorDegrees) <= HEADING_TOLERANCE_DEGREES;

        // Fire kicker when trigger is pulled and targets are ready
        boolean fireTrigger = gamepad.left_trigger > 0.2 || gamepad.right_trigger > 0.2;
        if (fireTrigger && velocityOnTarget && headingOnTarget) {
            shooterSubsystem.runKickerPower(1.0);
            kickerRunning = true;
        } else if (kickerRunning) {
            shooterSubsystem.stopKicker();
            kickerRunning = false;
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
        driveSubsystem.drive(0, 0, 0);
        kickerRunning = false;
    }

    private static double normalizeAngleRadians(double angleRad) {
        double a = angleRad % (2.0 * Math.PI);
        if (a > Math.PI) {
            return a - 2.0 * Math.PI;
        } else if (a < -Math.PI) {
            return a + 2.0 * Math.PI;
        }
        return a;
    }
}
