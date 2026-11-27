package org.firstinspires.ftc.teamcode.pedroPathing.nationals_paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Blue 2 Alliance Park Autonomous.
 */
@SuppressWarnings("unused")
public class BlueFlowerShootParkPath {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(81.6355, 130.9294, 270);
    private final Pose descore1 = poseFactory.of(81.4337, 111.6954, 270);
    private final Pose descore2 = poseFactory.of(112.4796, 111.8483, 270);
    private final Pose descore3 = poseFactory.of(112.4898, 94.873, 0);
    private final Pose descoreExtract = poseFactory.of(125.2, 94.8288, 0);
    private final Pose descorerBack = poseFactory.of(112.6558, 94.6549, 0);
    private final Pose shoot2Prep1 = poseFactory.of(112.2569, 120.0912, 270);
    private final Pose shootPrep2 = poseFactory.of(81.6141, 119.6844, 90);
    private final Pose shootPrep3 = poseFactory.of(81.7569, 22.6382, 90);
    private final Pose park = poseFactory.of(129.1073, 48.1554, 90);

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

    public Path shoot2Prep1() {
        return Paths.line(descorerBack, shoot2Prep1).constant(shoot2Prep1);
    }

    public Path shootPrep2() {
        return Paths.line(shoot2Prep1, shootPrep2).constant(shootPrep2);
    }

    public Path shootPrep3() {
        return Paths.line(shootPrep2, shootPrep3).constant(shootPrep3);
    }

    public Path park() {
        return Paths.line(shootPrep3, park).constant(park);
    }
}
