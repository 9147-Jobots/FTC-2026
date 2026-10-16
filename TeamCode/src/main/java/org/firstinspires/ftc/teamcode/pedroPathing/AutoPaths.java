package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Exported path definitions from the Pedro Pathing visualizer.
 */
@SuppressWarnings("unused")
public class AutoPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(111.9197, 132.7114, -90);
    public final Pose path1 = poseFactory.of(111.6287, 40.6357, -90);
    public final Pose point2 = poseFactory.of(131.39, 40.4243, 179.3871);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).reverseTangent();
    }
}
