package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;

@TeleOp(name="Main Command TeleOp", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Reset scheduler state for this run
        CommandScheduler.getInstance().reset();

        // One seamless single line handles construction, setup, and auto-registration!
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Schedule our default driving command to process teleop gamepad inputs
        CommandScheduler.getInstance().schedule(new TeleOpDriveCommand(drive, gamepad1));

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // One simple call automatically runs all active subsystems and scheduled commands
            CommandScheduler.getInstance().run();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }
    }
}
