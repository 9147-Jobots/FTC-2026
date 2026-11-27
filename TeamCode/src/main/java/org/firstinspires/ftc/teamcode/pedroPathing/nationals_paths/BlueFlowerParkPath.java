package org.firstinspires.ftc.teamcode.pedroPathing.nationals_paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Blue 2 Alliance Park Autonomous.
 */
@SuppressWarnings("unused")
public class BlueFlowerParkPath {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(81.6355, 130.9294, 270);
    private final Pose descore1 = poseFactory.of(81.4337, 111.6954, 270);
    private final Pose descore2 = poseFactory.of(112.4796, 111.8483, 270);
    private final Pose descore3 = poseFactory.of(112.4898, 94.873, 0);
    private final Pose descoreExtract = poseFactory.of(125.2, 94.8288, 0);
    private final Pose descorerBack = poseFactory.of(112.6558, 94.6549, 0);
    private final Pose park = poseFactory.of(128.9055, 44.3202, 0);

    public Path descore1() {
        return Paths.line(start, descore1).linear(start, descore1);
    }

    public Path descore2() {
        return Paths.line(descore1, descore2).constant(descore2);
    }

    public Path descore3() {
        return Paths.line(descore2, descore3).constant(descore3);
    }

    public Path descoreExtract() {
        return Paths.line(descore3, descoreExtract).constant(descoreExtract);
    }

    public Path descorerBack() {
        return Paths.line(descoreExtract, descorerBack).constant(descorerBack);
    }

    public Path park() {
        return Paths.line(descorerBack, park).constant(park);
    }
}
