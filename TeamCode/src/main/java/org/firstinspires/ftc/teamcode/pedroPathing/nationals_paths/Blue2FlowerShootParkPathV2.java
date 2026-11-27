package org.firstinspires.ftc.teamcode.pedroPathing.nationals_paths;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Blue 2 Alliance Park Autonomous.
 */
@SuppressWarnings("unused")
public class Blue2FlowerShootParkPathV2 {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(82.2411, 7.7981, 90);
    private final Pose descore1 = poseFactory.of(82.4429, 31.3573, 90);
    private final Pose descore2 = poseFactory.of(93.909, 31.1065, 90);
    private final Pose descore3 = poseFactory.of(93.9192, 27.0499, 270);
    private final Pose descoreExtract = poseFactory.of(93.9145, 16.3, 270);
    private final Pose descorerBack = poseFactory.of(94.0852, 27.4374, 270);
    private final Pose shoot2Prep1 = poseFactory.of(100.15, 27.44, 90);
    private final Pose park = poseFactory.of(129.1073, 27.97, 90);

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

    public Path park() {
        return Paths.line(shoot2Prep1, park).constant(park);
    }
}
