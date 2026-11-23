package org.firstinspires.ftc.teamcode.pedroPathing.paths.flower;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Blue 2 Alliance Park Autonomous.
 */
@SuppressWarnings("unused")
public class Blue2Flower {
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(132.4702, 9.0388, 90);
    private final Pose blue2ToFlower = poseFactory.of(94.7, 24.8388, 270);
    private final Pose blue2FlowerIntake = poseFactory.of(94.7, 16, -90);

    public Path blue2ToFlower() {return Paths.line(start, blue2ToFlower).linear(start, blue2ToFlower);}
    public Path blue2FlowerIntake() {return Paths.line(blue2ToFlower, blue2FlowerIntake).tangent();}
}
