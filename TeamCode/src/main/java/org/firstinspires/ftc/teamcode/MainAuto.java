package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.Auto.AutoMove;
import org.firstinspires.ftc.teamcode.framework.CommandScheduler;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

@Autonomous(name="Main Command Auto", group="Linear OpMode")
@SuppressWarnings("unused")
public class MainAuto extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Reset scheduler state for this run
        CommandScheduler.getInstance().reset();

        Odometry odometry = new Odometry(hardwareMap, telemetry);
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Schedule autonomous routine (drives 1 meter forward)
        CommandScheduler.getInstance().schedule(new AutoMove(drive, odometry, 1.0));

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.update();
        }
    }
}


