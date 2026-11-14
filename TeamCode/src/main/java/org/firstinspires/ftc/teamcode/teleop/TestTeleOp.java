package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TeleOp.AutoAimAndShootCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpRunIndexerCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpRunIntakeCommand;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;
import org.firstinspires.ftc.teamcode.vision.AprilTagTargetGroup;
import org.firstinspires.ftc.teamcode.vision.VisionTargetResult;

@TeleOp(name="Test Command TeleOp", group="Linear OpMode")
@SuppressWarnings({"unused", "ReassignedVariable"})
public class TestTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Reset scheduler state for this run
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        IndexerSubsystem indexer = new IndexerSubsystem(hardwareMap, telemetry);
        AprilTagVisionSubsystem vision = new AprilTagVisionSubsystem(hardwareMap, telemetry);

        ShooterLookupTable lookupTable = new ShooterLookupTable();

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepad1));
        intake.setDefaultCommand(new TeleOpRunIntakeCommand(intake, gamepad1));
        indexer.setDefaultCommand(new TeleOpRunIndexerCommand(indexer, gamepad1));

        AutoAimAndShootCommand autoAimCommand = new AutoAimAndShootCommand(
                shooter, drive, indexer, vision, gamepad1, lookupTable
        );

        // Pre-match Alliance selection loop
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) {
                vision.setActiveAlliance(AprilTagTargetGroup.Alliance.BLUE);
            } else if (gamepad1.b) {
                vision.setActiveAlliance(AprilTagTargetGroup.Alliance.RED);
            }

            telemetry.addData("Status", "Test TeleOp Initialized - Press START when ready");
            telemetry.addData("Selected Alliance", "%s (Press X for BLUE, B for RED)", vision.getActiveAlliance());
            telemetry.addData("Auto-Aim Control", "Hold Gamepad 1 Right Trigger to Auto-Aim & Shoot");
            telemetry.update();

            sleep(20);
        }

        runtime.reset();

        boolean autoAimActive = false;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Activate Auto-Aim while Right Trigger is held
            boolean triggerPressed = gamepad1.right_trigger > 0.2;
            if (triggerPressed && !autoAimActive) {
                CommandScheduler.getInstance().schedule(autoAimCommand);
                autoAimActive = true;
            } else if (!triggerPressed && autoAimActive) {
                CommandScheduler.getInstance().cancel(autoAimCommand);
                autoAimActive = false;
            }

            // One simple call automatically runs all active subsystems and scheduled commands
            CommandScheduler.getInstance().run();

            VisionTargetResult targetResult = vision.getProjectedTargetResult();
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Auto Aim Active", autoAimActive);
            if (autoAimActive && targetResult.isTargetDetected()) {
                telemetry.addData("Target Group", targetResult.getGroup());
                telemetry.addData("Target Dist (M)", "%4.2f", targetResult.getDistanceMeters());
                telemetry.addData("Yaw Error (Deg)", "%4.1f°", targetResult.getYawDegrees());
            }

            telemetry.update();
        }

        // Clean up vision portal when OpMode ends
        vision.stopVision();
    }
}
