package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Static container used to store and transfer the robot field position
 * from Autonomous to TeleOp across OpMode transitions.
 */
public class PoseStorage {

    /**
     * Stores the latest robot field pose (in inches and radians).
     * Set by Autonomous OpModes upon completion, and consumed by TeleOp Odometry initialization.
     */
    public static Pose currentPose = null;

    /**
     * Clears the stored pose state.
     */
    public static void clear() {
        currentPose = null;
    }
}
