package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing v3.0 Constants configuration for Mecanum drivetrain, goBilda Pinpoint, and Foresight algorithm.
 */
@SuppressWarnings("unused")
public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left_drive");
        c.backLeftName.set("back_left_drive");
        c.frontRightName.set("front_right_drive");
        c.backRightName.set("back_right_drive");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odometry");
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        c.xPodOffset.set(155.0);
        c.yPodOffset.set(-168.0);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.resetMode.set(PinpointLocalizer.ResetMode.NONE);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.headingFeedback.set(Controller.pid(2.5, 0.0, 0.05));
        c.forwardTranslational.set(Controller.pid(0.12, 0.0, 0.01));
        c.strafeTranslational.set(Controller.pid(0.12, 0.0, 0.01));
        c.headingStaticFF.set(Controller.zero);
        c.brake.set(Controller.zero);
        c.coast.set(Controller.zero);

        c.holdPointTranslationalScaling.set(1.0);
        c.holdPointHeadingScaling.set(1.0);
        c.maxBrakingPower.set(1.0);

        c.maxPathSpeed.set(0.8);
        c.maxVelocityConstraint.set(60.0);
        c.maxAccelerationConstraint.set(50.0);
        c.maxDecelerationConstraint.set(50.0);
        c.maxDecelerationScale.set(1.0);
        c.brakeAggression.set(1.0);
        c.coastDownToVelocity.set(2.0);

        c.headingDeviationTolerance.set(Math.toRadians(2.0));
        c.translationalDeviationTolerance.set(1.2);
        c.brakeAtEnd.set(true);
        c.pathSkip.set(false);
        c.headingDriveRatio.set(1.0);

        c.linearBrakeCoefficients.set(Matrix.zero(2));
        c.quadraticBrakeCoefficients.set(Matrix.zero(2));
        c.headingBrakeCoefficients.set(Vector2D.zero());
        c.cosineScale.set(true);

        c.maxAchievableForwardVelocity.set(70.0);
        c.maxAchievableStrafeVelocity.set(60.0);
        c.naturalForwardDeceleration.set(50.0);
        c.naturalStrafeDeceleration.set(50.0);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
            new PinpointLocalizer(hardwareMap, localizerConfig),
            new Mecanum(hardwareMap, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
}
