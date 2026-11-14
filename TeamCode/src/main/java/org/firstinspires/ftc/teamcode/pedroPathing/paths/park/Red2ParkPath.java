package org.firstinspires.ftc.teamcode.pedroPathing.paths.park;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Path definitions for Red 2 Alliance Park Autonomous (Mirrored across both X and Y axes at field center 70.75 inches).
 */
@SuppressWarnings("unused")
public class Red2ParkPath {

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);

    public final Pose start = poseFactory.of(132.4702, 9.0388, 90);
    public final Pose point1 = poseFactory.of(132.4049, 25.2338, 90);

    public Path path1() {
        return Paths.line(start, point1).linear(start, point1);
    }
}
