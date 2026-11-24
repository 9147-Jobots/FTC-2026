package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.TeleOp.AutoAimAndShootCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.AimSourceSelector;
import org.firstinspires.ftc.teamcode.util.LynxUtil;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.vision.AprilTagTargetGroup;
import org.firstinspires.ftc.teamcode.vision.VisionTargetResult;

import java.util.Locale;

/**
 * Dedicated Calibration OpMode for mapping empirical distance data points into ShooterLookupTable.
 * Controls:
 * - D-Pad UP / DOWN: Increments / Decrements target flywheel velocity (+/- 25 TPS)
 * - Left / Right Bumpers: Adjusts hood angle (+/- 5.0 degrees)
 * - Y / X Buttons: Set hood to Max Position (270°) / Min Position (140°)
 * - Right Trigger: Runs Flywheel at target speed, then automatically runs Kicker & Indexer after 3 seconds or when shooter.isReady()
 * - Left Trigger: Runs Intake AND Indexer simultaneously to collect/load game elements
 * - Init Phase: Gamepad 1 X (BLUE Alliance) / B (RED Alliance)
 */
@TeleOp(name="Tune TeleOp", group="Linear OpMode")
@SuppressWarnings({"unused", "ReassignedVariable"})
public class TuneTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime rightTriggerTimer = new ElapsedTime();

    private double targetVelocityTPS = 1800.0; // Initial target velocity in ticks per second
    private static final double VELOCITY_STEP_TPS = 25.0;
    private static final double HOOD_STEP_DEGREES = 5.0;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastY = false;
    private boolean lastX = false;

    private boolean rightTriggerHeld = false;
    private boolean leftTriggerHeld = false;

    @Override
    public void runOpMode() {
        // Enable auto bulk caching on all REV LynxModules for optimized loop latency
        LynxUtil.enableAutoBulkCaching(hardwareMap);

        CommandScheduler.getInstance().reset();

        // Initialize Odometry with Blue Auto Shoot start pose: (85.87", 132.27", 270.0°)
        Pose2D blueShootStartPose = new Pose2D(DistanceUnit.INCH, 85.8713, 132.2699, AngleUnit.DEGREES, 270.0);
        Odometry odometry = new Odometry(hardwareMap, telemetry, blueShootStartPose);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        drive.setExternalHeadingSupplier(() -> odometry.getPose().getHeading(AngleUnit.RADIANS));
        drive.resetHeading();

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        shooter.setHoodAngleDegrees(180.0);
        IndexerSubsystem indexer = new IndexerSubsystem(hardwareMap, telemetry);
        AprilTagVisionSubsystem vision = new AprilTagVisionSubsystem(hardwareMap, telemetry);

        ShooterLookupTable lookupTable = new ShooterLookupTable();
        AutoAimAndShootCommand autoAimCommand = new AutoAimAndShootCommand(
                shooter, drive, indexer, intake, vision, odometry, gamepad1, lookupTable
        );

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepad1));

        // Pre-match Alliance selection loop
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                vision.setActiveAlliance(AprilTagTargetGroup.Alliance.BLUE);
            } else if (gamepad1.b) {
                vision.setActiveAlliance(AprilTagTargetGroup.Alliance.RED);
            }

            telemetry.addData("Status", "Tuning TeleOp Initialized - Press START when ready");
            telemetry.addData("Selected Alliance", "%s (Press X for BLUE, B for RED)", vision.getActiveAlliance());
            telemetry.addData("Tuning Help", "D-Pad Up/Down=TPS +/-25, Bumpers=Hood +/-5°, Hold B=Auto Aim & Shoot");
            telemetry.update();

            sleep(20);
        }

        runtime.reset();
        boolean autoAimActive = false;

        while (opModeIsActive()) {
            // 0. Auto Aim & Shoot Command Trigger (Hold Gamepad 1 B button)
            boolean autoAimPressed = gamepad1.b;
            if (autoAimPressed && !autoAimActive) {
                CommandScheduler.getInstance().schedule(autoAimCommand);
                autoAimActive = true;
            } else if (!autoAimPressed && autoAimActive) {
                CommandScheduler.getInstance().cancel(autoAimCommand);
                autoAimActive = false;
            }

            // 1. Manual Flywheel Target Velocity Adjustment (Gamepad 1 D-Pad UP/DOWN)
            if (gamepad1.dpad_up && !lastDpadUp) {
                targetVelocityTPS += VELOCITY_STEP_TPS;
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                targetVelocityTPS = Math.max(0.0, targetVelocityTPS - VELOCITY_STEP_TPS);
            }
            lastDpadDown = gamepad1.dpad_down;

            // 2. Manual Hood Angle Adjustment (Bumpers +/-5°, Y/X Max/Min)
            if (gamepad1.left_bumper && !lastLeftBumper) {
                shooter.adjustHoodAngleDegrees(HOOD_STEP_DEGREES);
            }
            lastLeftBumper = gamepad1.left_bumper;

            if (gamepad1.right_bumper && !lastRightBumper) {
                shooter.adjustHoodAngleDegrees(-HOOD_STEP_DEGREES);
            }
            lastRightBumper = gamepad1.right_bumper;

            if (gamepad1.y && !lastY) {
                shooter.setHoodMax();
            }
            lastY = gamepad1.y;

            if (gamepad1.x && !lastX) {
                shooter.setHoodMin();
            }
            lastX = gamepad1.x;

            // 3. Right Trigger: Run Flywheel at target velocity, then Kicker (pulsing), Indexer & Intake after 3 seconds or when ready
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.2;
            if (rightTriggerPressed) {
                if (!rightTriggerHeld) {
                    rightTriggerHeld = true;
                    rightTriggerTimer.reset();
                }

                shooter.runFlywheelVelocity(targetVelocityTPS);

                if (rightTriggerTimer.seconds() >= 3.0 || shooter.isReady()) {
                    shooter.runKickerPower(0.7);
                    indexer.runIndexer(1.0);
                    intake.runIntake();
                }
            } else {
                if (rightTriggerHeld) {
                    shooter.stopShooter();
                    shooter.stopKicker();
                    indexer.stopIndexer();
                    intake.stopIntake();
                    rightTriggerHeld = false;
                }
            }

            // 4. Left Trigger: Run Intake AND Indexer simultaneously to collect/load test elements
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.2;
            if (leftTriggerPressed) {
                if (!leftTriggerHeld) {
                    leftTriggerHeld = true;
                }
                intake.runIntake();
                indexer.runIndexer(1.0);
            } else {
                if (leftTriggerHeld) {
                    intake.stopIntake();
                    indexer.stopIndexer();
                    leftTriggerHeld = false;
                }
            }

            // Run command scheduler periodic routines
            CommandScheduler.getInstance().run();

            // 5. Calibration Dashboard & Copy-Paste Code Telemetry
            VisionTargetResult targetResult = vision.getProjectedTargetResult();
            AimSourceSelector.AimSource selectedSource = AimSourceSelector.selectAimSource(vision);

            double visionDistMeters = targetResult.getDistanceMeters();
            double currentHoodDeg = shooter.getHoodAngleDegrees();
            double actualVelocityTPS = shooter.getFlywheelVelocity();

            Pose2D pose = odometry.getPose();
            double rx = pose.getX(DistanceUnit.METER);
            double ry = pose.getY(DistanceUnit.METER);
            boolean isRed = vision.getActiveAlliance() == AprilTagTargetGroup.Alliance.RED;
            double hx = isRed ?
                    org.firstinspires.ftc.teamcode.pedroPathing.Constants.RED_HIVE_1_CENTER_X_METERS :
                    org.firstinspires.ftc.teamcode.pedroPathing.Constants.BLUE_HIVE_1_CENTER_X_METERS;
            double hy = isRed ?
                    org.firstinspires.ftc.teamcode.pedroPathing.Constants.RED_HIVE_1_CENTER_Y_METERS :
                    org.firstinspires.ftc.teamcode.pedroPathing.Constants.BLUE_HIVE_1_CENTER_Y_METERS;
            double odomDistMeters = Math.hypot(hx - rx, hy - ry);

            telemetry.addLine("=== CALIBRATION DASHBOARD ===");
            telemetry.addData("Auto Aim Active (Hold B)", autoAimActive);
            if (autoAimActive) {
                telemetry.addData("Shooter State Machine", autoAimCommand.getShooterState());
                telemetry.addData("Aim Source Active", autoAimCommand.getActiveAimSource());
            }
            telemetry.addData("Odometry Pose (Inches)", "X: %4.1fin, Y: %4.1fin, Heading: %4.1f°",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Odom Hive Dist (m)", "%4.2fm (%4.1fin)", odomDistMeters, odomDistMeters / 0.0254);
            telemetry.addData("Position Source Selected", selectedSource.isVision() ? "VISION" : "ODOMETRY");
            telemetry.addData("Vision Target Detected", targetResult.isTargetDetected());
            if (targetResult.isTargetDetected()) {
                telemetry.addData("Tag Group / Primary ID", "%s (#%d)", targetResult.getGroup(), targetResult.getPrimaryTagId());
                telemetry.addData("Vision Floor Dist (m)", "%4.2fm (%4.1fin)", visionDistMeters, visionDistMeters / 0.0254);
                telemetry.addData("Center Yaw Error", "%4.1f°", targetResult.getYawDegrees());
            } else {
                telemetry.addData("Vision Status", "No Target Detected for %s", vision.getActiveAlliance());
            }

            telemetry.addLine("--- SHOOTER MANUAL TUNING ---");
            telemetry.addData("Target Velocity (TPS)", "%4.0f TPS", targetVelocityTPS);
            telemetry.addData("Actual Flywheel (TPS)", "%4.0f TPS", actualVelocityTPS);
            telemetry.addData("Shooter Ready (isReady)", shooter.isReady());
            telemetry.addData("Current Hood Angle", "%4.1f°", currentHoodDeg);

            if (targetResult.isTargetDetected()) {
                telemetry.addLine("--- COPY-PASTE FOR ShooterLookupTable 2D ---");
                telemetry.addLine(String.format(Locale.US, "addVisionPoint2D(%.2f, %.0f, %.1f);",
                        visionDistMeters, targetVelocityTPS, currentHoodDeg));
            }

            telemetry.update();
        }

        // Cleanup resources on stop
        shooter.stopShooter();
        shooter.stopKicker();
        indexer.stopIndexer();
        vision.stopVision();
    }
}
