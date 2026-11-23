package org.firstinspires.ftc.teamcode.pedroPathing.paths.flower;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Blue 2 Alliance Park Autonomous.
 */
@SuppressWarnings("unused")
public class BlueFlower {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(111.9197, 132.7114, 90);
    private final Pose blueToFlower = poseFactory.of(116.5, 94.6, 0);
    private final Pose blueFlowerExtract = poseFactory.of(125.5, 94.6, 0);

    public Path blueToFlower() {return Paths.line(start, blueToFlower).linear(start, blueToFlower);}

    public Path blueFlowerExtract() {return Paths.line(blueToFlower, blueFlowerExtract).tangent();}
}
