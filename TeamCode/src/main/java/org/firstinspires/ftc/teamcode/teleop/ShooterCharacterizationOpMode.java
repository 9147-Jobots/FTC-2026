package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Battery;
import org.firstinspires.ftc.teamcode.util.LynxUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Empirical Characterization OpMode for Shooter Flywheel Mechanism.
 * Steps through fixed voltages (2.0V to 10.0V), records steady-state velocity at each voltage step,
 * and performs linear regression (Voltage vs Steady-State Velocity TPS) to fit:
 *   Volts = kS + kV * TargetVelocity
 * <p>
 * Results (kS and kV) are logged to telemetry for live copy-pasting into ShooterSubsystem @Config static fields.
 */
@TeleOp(name = "Shooter Characterization", group = "Tuning")
public class ShooterCharacterizationOpMode extends LinearOpMode {

    private static class DataPoint {
        final double voltage;
        final double velocityTPS;

        DataPoint(double voltage, double velocityTPS) {
            this.voltage = voltage;
            this.velocityTPS = velocityTPS;
        }
    }

    @Override
    public void runOpMode() {
        LynxUtil.enableAutoBulkCaching(hardwareMap);

        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Battery battery = new Battery(hardwareMap);
        ElapsedTime timer = new ElapsedTime();

        telemetry.addLine("=== SHOOTER CHARACTERIZATION OPMODE ===");
        telemetry.addLine("Ensure robot is safe and flywheel is clear.");
        telemetry.addLine("Press START then Gamepad 1 A button to run characterization.");
        telemetry.update();

        waitForStart();

        List<DataPoint> points = new ArrayList<>();
        boolean runCharacterization = false;

        while (opModeIsActive() && !runCharacterization) {
            if (gamepad1.a) {
                runCharacterization = true;
            }

            telemetry.addLine("=== READY FOR CHARACTERIZATION ===");
            telemetry.addLine("Press Gamepad 1 A button to begin voltage stepping...");
            telemetry.addData("Battery Voltage", "%4.2f V", battery.volts());
            telemetry.update();
            sleep(20);
        }

        if (runCharacterization && opModeIsActive()) {
            double[] targetVoltages = {2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};

            for (double vTarget : targetVoltages) {
                if (!opModeIsActive()) break;

                // Step 1: Apply fixed voltage for 2.0 seconds to reach steady-state velocity
                timer.reset();
                while (opModeIsActive() && timer.seconds() < 2.0) {
                    double vBatt = battery.volts();
                    double power = Math.min(1.0, Math.max(0.0, vTarget / vBatt));
                    shooter.setPower(power);

                    telemetry.addLine("=== CHARACTERIZING... ===");
                    telemetry.addData("Target Voltage", "%4.1f V", vTarget);
                    telemetry.addData("Current Power", "%4.2f", power);
                    telemetry.addData("Actual Velocity", "%4.0f TPS", shooter.getVelocity());
                    telemetry.addData("Phase", "Settling steady state (2.0s)...");
                    telemetry.update();
                    sleep(20);
                }

                // Step 2: Sample steady-state velocity over 1.0 second window
                timer.reset();
                double velSum = 0.0;
                int count = 0;

                while (opModeIsActive() && timer.seconds() < 1.0) {
                    double vBatt = battery.volts();
                    double power = Math.min(1.0, Math.max(0.0, vTarget / vBatt));
                    shooter.setPower(power);

                    velSum += shooter.getVelocity();
                    count++;

                    telemetry.addLine("=== CHARACTERIZING... ===");
                    telemetry.addData("Target Voltage", "%4.1f V", vTarget);
                    telemetry.addData("Sampling Velocity", "%4.0f TPS", shooter.getVelocity());
                    telemetry.addData("Phase", "Sampling (1.0s window)...");
                    telemetry.update();
                    sleep(20);
                }

                if (count > 0) {
                    double avgVel = velSum / count;
                    points.add(new DataPoint(vTarget, avgVel));
                }
            }

            shooter.setPower(0.0);

            // Calculate Linear Regression (Volts vs Velocity TPS)
            int n = points.size();
            if (n >= 2) {
                double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0, sumYY = 0;
                for (DataPoint pt : points) {
                    double x = pt.velocityTPS; // x = velocity (TPS)
                    double y = pt.voltage;     // y = voltage (Volts)
                    sumX += x;
                    sumY += y;
                    sumXY += x * y;
                    sumXX += x * x;
                    sumYY += y * y;
                }

                double denominator = (n * sumXX - sumX * sumX);
                double kV = (denominator != 0) ? (n * sumXY - sumX * sumY) / denominator : 0.0;
                double kS = (sumY - kV * sumX) / n;

                double numR = (n * sumXY - sumX * sumY);
                double denR = Math.sqrt((n * sumXX - sumX * sumX) * (n * sumYY - sumY * sumY));
                double rSquared = (denR != 0) ? Math.pow(numR / denR, 2) : 0.0;

                // Display Characterization Results
                while (opModeIsActive()) {
                    telemetry.addLine("=== CHARACTERIZATION COMPLETE ===");
                    telemetry.addLine("--- COPY-PASTE GAINS INTO ShooterSubsystem ---");
                    telemetry.addLine(String.format(Locale.US, "public static double kS = %.4f;", kS));
                    telemetry.addLine(String.format(Locale.US, "public static double kV = %.6f;", kV));
                    telemetry.addLine(String.format(Locale.US, "R^2 Fit Quality: %.4f", rSquared));
                    telemetry.addLine("---------------------------------------------");
                    telemetry.addLine("Data Points (Applied V -> Steady Velocity TPS):");
                    for (DataPoint pt : points) {
                        telemetry.addLine(String.format(Locale.US, "  %4.1f V  ->  %5.0f TPS", pt.voltage, pt.velocityTPS));
                    }
                    telemetry.update();
                    sleep(100);
                }
            }
        }

        shooter.setPower(0.0);
    }
}
