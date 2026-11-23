package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/**
 * Utility helper for managing REV Hub LynxModules, bulk caching modes, and hardware bus optimizations.
 */
public class LynxUtil {

    private LynxUtil() {}

    /**
     * Enables automatic bulk caching mode (LynxModule.BulkCachingMode.AUTO) on all REV Control Hubs / Expansion Hubs.
     * In AUTO mode, bulk read cache is cleared automatically once per loop cycle upon hardware read request.
     *
     * @param hardwareMap Active OpMode HardwareMap instance.
     */
    public static void enableAutoBulkCaching(HardwareMap hardwareMap) {
        if (hardwareMap == null) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
