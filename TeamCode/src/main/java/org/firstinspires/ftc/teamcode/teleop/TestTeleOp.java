package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TeleOp.AutoAimAndShootCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpRunIndexerCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpRunShooterCommand;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IndexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterLookupTable;

@TeleOp(name="Test Command TeleOp", group="Linear OpMode")
@SuppressWarnings({"unused", "ReassignedVariable", "ConstantConditions", "LoopConditionIfSeverity"})
public class TestTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    // Target High Goal coordinates in meters for testing
    private static final double TARGET_GOAL_X = 0.0;
    private static final double TARGET_GOAL_Y = 3.6;

    @Override
    public void runOpMode() {
        // Reset scheduler state for this run
        CommandScheduler.getInstance().reset();

        // Subsystems required for Shooter, Indexer, and Auto-Aim testing
        Odometry odometry = new Odometry(hardwareMap, telemetry);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        IndexerSubsystem indexer = new IndexerSubsystem(hardwareMap, telemetry);

        // Shooter lookup table with linear interpolation presets
        ShooterLookupTable lookupTable = new ShooterLookupTable();

        // Default commands
        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepad1));
        shooter.setDefaultCommand(new TeleOpRunShooterCommand(shooter, gamepad1));
        indexer.setDefaultCommand(new TeleOpRunIndexerCommand(indexer, gamepad1));

        // Auto Aim and Shoot Command for testing
        AutoAimAndShootCommand autoAimCommand = new AutoAimAndShootCommand(
                shooter, drive, odometry, gamepad1, TARGET_GOAL_X, TARGET_GOAL_Y, lookupTable
        );

        telemetry.addData("Status", "Test TeleOp Initialized (Shooter & Indexer Testing)");
        telemetry.addData("Auto-Aim", "Press Gamepad 1 'A' to toggle Auto-Aim mode");
        telemetry.update();

        waitForStart();
        runtime.reset();

        boolean lastAState = false;
        boolean autoAimActive = false;

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Toggle Auto-Aim mode on Button A press
            boolean currentA = gamepad1.a;
            if (currentA && !lastAState) {
                autoAimActive = !autoAimActive;
                if (autoAimActive) {
                    CommandScheduler.getInstance().schedule(autoAimCommand);
                } else {
                    CommandScheduler.getInstance().cancel(autoAimCommand);
                }
            }
            lastAState = currentA;

            // Run all active subsystems and scheduled commands
            CommandScheduler.getInstance().run();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Auto Aim Mode Active", autoAimActive);
            telemetry.update();
        }
    }
}
