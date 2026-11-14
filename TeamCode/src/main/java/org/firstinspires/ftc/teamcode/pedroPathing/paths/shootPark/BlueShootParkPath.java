package org.firstinspires.ftc.teamcode.pedroPathing.paths.shootPark;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Blue Alliance Shoot & Park Autonomous.
 */
@SuppressWarnings("unused")
public class BlueShootParkPath {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(85.8713, 132.2699, 270);
    private final Pose path1 = poseFactory.of(112.5117, 112.8206, 230);
    private final Pose point2Start = poseFactory.of(112.5117, 112.8206, 270);
    private final Pose point2 = poseFactory.of(112.8471, 49.9165, 270);
    private final Pose point3Start = poseFactory.of(112.8471, 49.9165, 270);
    private final Pose point3 = poseFactory.of(132.046, 49.8565, 270);

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
