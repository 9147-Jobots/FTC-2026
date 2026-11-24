package org.firstinspires.ftc.teamcode.pedroPathing.paths.flower;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Path definitions for Blue 2 Alliance Park Autonomous.
 */
@SuppressWarnings("unused")
public class BlueFlowerPath {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(85.2, 132.7144, 270);
    private final Pose blueToFlower = poseFactory.of(110, 95, 360);
    private final Pose blueFlowerBack = poseFactory.of(115, 95, 0);
    private final Pose blueToFlowerSegment1Start = poseFactory.of(110, 95, 270);
    private final Pose blueToFlowerSegment1End = poseFactory.of(110, 95, 270);
    private final Pose blueToFlowerSegment2Start = poseFactory.of(110, 95, 270);
    private final Pose blueToFlowerSegment2End = poseFactory.of(110, 95, 0);
    private final Pose blueFlowerExtract = poseFactory.of(130, 95, 0);



    public Path blueToFlower() {
        return Paths.line(start, blueToFlower).heading(Interpolator.piecewise().until(0.8993, Interpolator.linear(blueToFlowerSegment1Start, blueToFlowerSegment1End)).until(1, Interpolator.linear(blueToFlowerSegment2Start, blueToFlowerSegment2End)));
    }

    public Path blueFlowerBack() {
        return Paths.line(blueFlowerExtract, blueFlowerBack).tangent();
    }

    public Path blueFlowerExtract() {
        return Paths.line(blueToFlower, blueFlowerExtract).tangent().with(Constants.foresightConfig.maxPathSpeed.at(0.4));
    }
}
