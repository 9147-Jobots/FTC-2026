package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpRunIntakeCommand;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.vision.AprilTagTargetGroup;
import org.firstinspires.ftc.teamcode.vision.VisionTargetResult;

import java.util.Locale;

/**
 * Dedicated Calibration OpMode for mapping empirical distance data points into ShooterLookupTable.
 * Controls:
 * - D-Pad UP / DOWN: Increments / Decrements target flywheel velocity (+/- 50 TPS)
 * - Left / Right Bumpers: Adjusts hood angle (+/- 5.0 degrees)
 * - Y / X Buttons: Set hood to Max Position (300°) / Min Position (150°)
 * - Right Trigger: Spins flywheel at manually tuned target speed
 * - Left Trigger: Runs BOTH kicker motor AND indexer motor simultaneously to fire test shots
 * - Init Phase: Gamepad 1 X (BLUE Alliance) / B (RED Alliance)
 */
@TeleOp(name="Tune TeleOp", group="Linear OpMode")
@SuppressWarnings("unused")
public class TuneTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private double targetVelocityTPS = 1800.0; // Initial target velocity in ticks per second
    private static final double VELOCITY_STEP_TPS = 50.0;
    private static final double HOOD_STEP_DEGREES = 5.0;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastY = false;
    private boolean lastX = false;

    private boolean shooterRunning = false;
    private boolean feedRunning = false;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        IndexerSubsystem indexer = new IndexerSubsystem(hardwareMap, telemetry);
        AprilTagVisionSubsystem vision = new AprilTagVisionSubsystem(hardwareMap, telemetry);

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepad1));
        intake.setDefaultCommand(new TeleOpRunIntakeCommand(intake, gamepad1));

        // Pre-match Alliance selection loop
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                vision.setActiveAlliance(AprilTagTargetGroup.Alliance.BLUE);
            } else if (gamepad1.b) {
                vision.setActiveAlliance(AprilTagTargetGroup.Alliance.RED);
            }

            telemetry.addData("Status", "Tuning TeleOp Initialized - Press START when ready");
            telemetry.addData("Selected Alliance", "%s (Press X for BLUE, B for RED)", vision.getActiveAlliance());
            telemetry.addData("Tuning Help", "D-Pad Up/Down=TPS +/-50, Bumpers=Hood +/-5°");
            telemetry.update();

            sleep(20);
        }

        runtime.reset();

        while (opModeIsActive()) {
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

            // 3. Right Trigger: Run Flywheel at target velocity
            boolean rightTriggerPressed = gamepad1.right_trigger > 0.2;
            if (rightTriggerPressed) {
                shooter.runFlywheelVelocity(targetVelocityTPS);
                shooterRunning = true;
            } else if (shooterRunning) {
                shooter.stopShooter();
                shooterRunning = false;
            }

            // 4. Left Trigger: Run BOTH Kicker AND Pulsing Indexer simultaneously to feed test shots
            boolean leftTriggerPressed = gamepad1.left_trigger > 0.2;
            if (leftTriggerPressed) {
                shooter.runKickerPower(1.0);
                indexer.startPulsing(200.0, 150.0, 1.0);
                feedRunning = true;
            } else if (feedRunning) {
                shooter.stopKicker();
                indexer.stopIndexer();
                feedRunning = false;
            }

            // Run command scheduler periodic routines
            CommandScheduler.getInstance().run();

            // 5. Calibration Dashboard & Copy-Paste Code Telemetry
            VisionTargetResult targetResult = vision.getProjectedTargetResult();
            double distanceMeters = targetResult.getDistanceMeters();
            double currentHoodDeg = shooter.getHoodAngleDegrees();
            double actualVelocityTPS = shooter.getFlywheelVelocity();

            telemetry.addLine("=== CALIBRATION DASHBOARD ===");
            telemetry.addData("Vision Target Detected", targetResult.isTargetDetected());
            if (targetResult.isTargetDetected()) {
                telemetry.addData("Tag Group / Primary ID", "%s (#%d)", targetResult.getGroup(), targetResult.getPrimaryTagId());
                telemetry.addData("Floor Dist (M / Inches)", "%4.2fm (%4.1fin)", distanceMeters, distanceMeters / 0.0254);
                telemetry.addData("Center Yaw Error", "%4.1f°", targetResult.getYawDegrees());
            } else {
                telemetry.addData("Vision Status", "No Target Detected for %s", vision.getActiveAlliance());
            }

            telemetry.addLine("--- SHOOTER MANUAL TUNING ---");
            telemetry.addData("Target Velocity (TPS)", "%4.0f TPS", targetVelocityTPS);
            telemetry.addData("Actual Flywheel (TPS)", "%4.0f TPS", actualVelocityTPS);
            telemetry.addData("Current Hood Angle", "%4.1f°", currentHoodDeg);

            if (targetResult.isTargetDetected()) {
                telemetry.addLine("--- COPY-PASTE FOR ShooterLookupTable ---");
                telemetry.addLine(String.format(Locale.US, "addPoint(%.2f, %.0f, %.1f);",
                        distanceMeters, targetVelocityTPS, currentHoodDeg));
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
