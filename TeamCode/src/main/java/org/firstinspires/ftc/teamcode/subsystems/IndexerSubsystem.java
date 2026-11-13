package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;

/**
 * Subsystem encapsulating indexer motor mechanism and non-blocking pulse controller.
 */
@SuppressWarnings("unused")
public class IndexerSubsystem extends SubsystemBase {

    private final Telemetry telemetry;
    private final DcMotor indexerMotor;

    private final ElapsedTime pulseTimer = new ElapsedTime();
    private boolean isPulsing = false;
    private double pulseOnMs = 200.0;
    private double pulseOffMs = 150.0;
    private double pulsePower = 1.0;

    public IndexerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super();
        this.telemetry = telemetry;
        indexerMotor = hardwareMap.get(DcMotor.class, "indexer_motor");
        indexerMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /** Starts non-blocking indexer pulsing with custom ON/OFF durations (in ms) and power. */
    public void startPulsing(double onMs, double offMs, double power) {
        this.pulseOnMs = onMs;
        this.pulseOffMs = offMs;
        this.pulsePower = power;
        if (!isPulsing) {
            this.isPulsing = true;
            this.pulseTimer.reset();
        }
    }

    /** Starts non-blocking indexer pulsing with recommended defaults (200ms ON / 150ms OFF @ 1.0 power). */
    public void startPulsing() {
        startPulsing(200.0, 150.0, 1.0);
    }

    public boolean isPulsing() {
        return isPulsing;
    }

    @Override
    public void periodic() {
        if (isPulsing) {
            double totalCycleMs = pulseOnMs + pulseOffMs;
            double elapsedInCycleMs = pulseTimer.milliseconds() % totalCycleMs;

            if (elapsedInCycleMs < pulseOnMs) {
                indexerMotor.setPower(pulsePower);
            } else {
                indexerMotor.setPower(0.0);
            }
        }
        telemetry.addData("Indexer Power", "%4.2f", indexerMotor.getPower());
        telemetry.addData("Indexer Pulsing", isPulsing);
    }

    public void runIndexer() {
        runIndexer(1.0);
    }

    public void runIndexer(double power) {
        this.isPulsing = false;
        indexerMotor.setPower(power);
    }

    public void stopIndexer() {
        this.isPulsing = false;
        indexerMotor.setPower(0.0);
    }
}
