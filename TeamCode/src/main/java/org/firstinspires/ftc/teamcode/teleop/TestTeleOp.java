package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpRunIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpRunShooterCommand;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name="Test Command TeleOp", group="Linear OpMode")
@SuppressWarnings("unused")
public class TestTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Reset scheduler state for this run
        CommandScheduler.getInstance().reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry);
        AprilTagVisionSubsystem vision = new AprilTagVisionSubsystem(hardwareMap, telemetry);

        drive.setDefaultCommand(new TeleOpDriveCommand(drive, gamepad1));
        intake.setDefaultCommand(new TeleOpRunIntakeCommand(intake, gamepad1));
        shooter.setDefaultCommand(new TeleOpRunShooterCommand(shooter, gamepad1));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // One simple call automatically runs all active subsystems and scheduled commands
            CommandScheduler.getInstance().run();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }

        // Clean up vision portal when OpMode ends
        vision.stopVision();
    }
}
