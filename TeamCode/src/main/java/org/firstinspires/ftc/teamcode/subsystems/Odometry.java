package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;

public class Odometry extends SubsystemBase {

    private final Telemetry telemetry;

    private final GoBildaPinpointDriver odometry;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry) {
        super();

        this.telemetry = telemetry;

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");

        odometry.setOffsets(155, -168.0, DistanceUnit.MM);

        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odometry.resetPosAndIMU();
    }

    public Pose2D getPose() {
        return odometry.getPosition();
    }

    @Override
    public void periodic() {
        odometry.update();
        Pose2D position = getPose();
        telemetry.addData("Odometry X (M)", "%4.2f", position.getX(DistanceUnit.METER));
        telemetry.addData("Odometry Y (M)", "%4.2f", position.getY(DistanceUnit.METER));
        telemetry.addData("Heading (Deg)", "%4.2f", position.getHeading(AngleUnit.DEGREES));
    }
}
