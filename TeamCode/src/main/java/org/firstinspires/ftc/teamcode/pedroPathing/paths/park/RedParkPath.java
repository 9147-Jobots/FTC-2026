package org.firstinspires.ftc.teamcode.pedroPathing.paths.park;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Red Alliance Park Autonomous (Mirrored across both X and Y axes at field center 70.75 inches).
 */
@SuppressWarnings("unused")
public class RedParkPath {

    // Mirrors across both X and Y axes (field center 70.75 inches) for Red alliance
    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

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
