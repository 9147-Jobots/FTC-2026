package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.framework.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

public class Odometry extends SubsystemBase {

    private final Telemetry telemetry;

    private final GoBildaPinpointDriver odometry;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
    }

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry, Pose2D defaultStartPose) {
        super();

        this.telemetry = telemetry;

        odometry = hardwareMap.get(GoBildaPinpointDriver.class, "odometry");

        odometry.setOffsets(6.170797272929995, -5.420950942152128, DistanceUnit.INCH);

        odometry.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        if (PoseStorage.currentPose != null) {
            odometry.setPosition(new Pose2D(
                DistanceUnit.INCH,
                PoseStorage.currentPose.x(),
                PoseStorage.currentPose.y(),
                AngleUnit.RADIANS,
                PoseStorage.currentPose.heading()
            ));
        } else if (defaultStartPose != null) {
            odometry.setPosition(defaultStartPose);
        } else {
            odometry.resetPosAndIMU();
        }
    }

    public Pose2D getPose() {
        return odometry.getPosition();
    }

    public void resetPose() {
        odometry.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public void resetPoseTo(double xInches, double yInches, double headingDegrees) {
        odometry.setPosition(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.DEGREES, headingDegrees));
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
