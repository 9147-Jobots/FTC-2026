package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Main Subsystem TeleOp", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private final List<Subsystem> subsystems = new ArrayList<>();

    @Override
    public void runOpMode() {
        // Register subsystems
        subsystems.add(new DriveSubsystem(hardwareMap, gamepad1, telemetry));

        // Initialize all registered subsystems
        for (Subsystem subsystem : subsystems) {
            subsystem.init();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Update all subsystems periodically
            for (Subsystem subsystem : subsystems) {
                subsystem.periodic();
            }

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }
    }
}
