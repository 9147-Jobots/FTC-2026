package org.firstinspires.ftc.teamcode.vision;

import java.util.HashMap;
import java.util.Map;

/**
 * Defines AprilTag groups for Blue and Red alliances and physical horizontal tag offsets (in inches)
 * relative to the projected target center of each group.
 * Group Geometry:
 * [Tag 1: -6.50"] -- 0.5" -- [Tag 2: -2.75"] ===== 2.25" Gap (Center = 0") ===== [Tag 3: +2.75"] -- 0.5" -- [Tag 4: +6.50"]
 */
@SuppressWarnings({"unused", "ReassignedVariable"})
public enum AprilTagTargetGroup {

    BLUE_GROUP_1(Alliance.BLUE, new int[]{38, 39, 40, 41}),
    BLUE_GROUP_2(Alliance.BLUE, new int[]{42, 43, 44, 45}),
    RED_GROUP_1(Alliance.RED,  new int[]{34, 35, 36, 37}),
    RED_GROUP_2(Alliance.RED,  new int[]{30, 31, 32, 33});

    public enum Alliance {
        BLUE, RED
    }

    private final Alliance alliance;
    private final int[] tagIds;
    private final Map<Integer, Double> tagOffsetInchesMap = new HashMap<>();

    AprilTagTargetGroup(Alliance alliance, int[] tagIds) {
        this.alliance = alliance;
        this.tagIds = tagIds;
        double[] offsets = new double[]{-6.50, -2.75, +2.75, +6.50};
        for (int i = 0; i < tagIds.length && i < offsets.length; i++) {
            tagOffsetInchesMap.put(tagIds[i], offsets[i]);
        }
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public int[] getTagIds() {
        return tagIds.clone();
    }

    public boolean containsTag(int tagId) {
        return tagOffsetInchesMap.containsKey(tagId);
    }

    /**
     * Returns the horizontal offset (in inches) from the detected AprilTag center to the group's projected center.
     */
    public double getTagHorizontalOffsetInches(int tagId) {
        Double offset = tagOffsetInchesMap.get(tagId);
        return offset != null ? offset : 0.0;
    }

    /**
     * Returns the horizontal offset (in meters) from the detected AprilTag center to the group's projected center.
     */
    public double getTagHorizontalOffsetMeters(int tagId) {
        return getTagHorizontalOffsetInches(tagId) * 0.0254;
    }

    /**
     * Finds the matching group for a given AprilTag ID, or null if none match.
     */
    public static AprilTagTargetGroup getGroupForTag(int tagId) {
        for (AprilTagTargetGroup group : values()) {
            if (group.containsTag(tagId)) {
                return group;
            }
        }
        return null;
    }
}
