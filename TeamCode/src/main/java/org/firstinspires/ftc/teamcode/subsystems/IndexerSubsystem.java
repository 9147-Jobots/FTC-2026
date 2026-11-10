package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;

/**
 * Subsystem encapsulating indexer motor mechanism.
 */
@SuppressWarnings("unused")
public class IndexerSubsystem extends SubsystemBase {
    private final Telemetry telemetry;
    private final DcMotor indexerMotor;

    public IndexerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        super();
        this.telemetry = telemetry;
        indexerMotor = hardwareMap.get(DcMotor.class, "indexer_motor");
        indexerMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        telemetry.addData("Indexer Power", "%4.2f", indexerMotor.getPower());
    }

    public void runIndexer() {
        runIndexer(1.0);
    }

    public void runIndexer(double power) {
        indexerMotor.setPower(power);
    }

    public void stopIndexer() {
        indexerMotor.setPower(0);
    }
}
