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
        c.frontRightName.set("front_right_drive");
        c.backLeftName.set("back_left_drive");
        c.backRightName.set("back_right_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odometry");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.649760028508704);
        c.yPodOffset.set(-5.333017063891794);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.15833874457625488);
                Controller secondaryTranslationalForward = Controller.proportional(0.0585019352317497);
                Controller primaryTranslationalLateral = Controller.proportional(0.38522824226117025);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14233154202729129);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01838565699161314));
                c.brake.set(Controller.proportionalFeedforward(0.015627808442871167));

                c.headingFeedback.set(Controller.proportional(3.075644815206101));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04454575932743435, 0.004732480462337032));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06116121113982052, 0.05982572299360239));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011527223069045939, 0.0012513509331924051));

                c.maxAchievableForwardVelocity.set(59.53007357980147);
                c.maxAchievableStrafeVelocity.set(48.16859043739426);
                c.naturalForwardDeceleration.set(38.40649034670365);
                c.naturalStrafeDeceleration.set(59.69283751551569);
            }
    );

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
            new PinpointLocalizer(hardwareMap, localizerConfig),
            new Mecanum(hardwareMap, drivetrainConfig),
            new Foresight(foresightConfig)
        );
    }
}
