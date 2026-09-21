package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TeleOp.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name="Main Command Auto", group="Linear OpMode")
public class MainAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Reset scheduler state for this run
        CommandScheduler.getInstance().reset();

        // One seamless single line handles construction, setup, and auto-registration!
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);
        //IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);
        //IndexerSubsystem indexer = new IndexerSubsystem(hardwareMap, telemetry)
        //ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap, telemetry)

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Schedule our default driving command to process teleop gamepad inputs
        CommandScheduler.getInstance().schedule(new TeleOpDriveCommand(drive, gamepad1));
        //CommandScheduler.getInstance().schedule(new TeleOpRunIntakeCommand(intake, gamepad1));

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // One simple call automatically runs all active subsystems and scheduled commands
            CommandScheduler.getInstance().run();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }
    }
}
