package org.firstinspires.ftc.teamcode.pedroPathing.paths.shootPark;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Blue 2 Alliance Shoot & Park Autonomous.
 */
@SuppressWarnings("unused")
public class Blue2ShootParkPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(132.4702, 9.0388, 90);
    private final Pose path1 = poseFactory.of(120.8824, 17.3261, 90);
    private final Pose point2Start = poseFactory.of(120.8824, 17.3261, 130);
    private final Pose point2 = poseFactory.of(110.2936, 25.3201, 130);
    private final Pose point3Start = poseFactory.of(110.2936, 25.3201, 90);
    private final Pose point3 = poseFactory.of(132.4049, 25.2338, 90);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point3Start, point3).linear(point3Start, point3);
    }
}
