package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Utility class to read robot battery voltage safely without introducing excessive I2C/CAN bus latency.
 * Throttles hardware voltage sensor reads to at most once every 200 ms and applies an Exponential Moving Average
 * (low-pass filter) to smooth out voltage spikes and noise.
 */
public class Battery {
    private static final double DEFAULT_READ_INTERVAL_MS = 200.0;
    private static final double DEFAULT_ALPHA = 0.2; // EMA smoothing factor (0.0 < alpha <= 1.0)
    private static final double DEFAULT_NOMINAL_VOLTAGE = 12.0;

    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private final double readIntervalMs;
    private final double alpha;

    private double filteredVoltage = DEFAULT_NOMINAL_VOLTAGE;
    private boolean initialized = false;

    public Battery(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_READ_INTERVAL_MS, DEFAULT_ALPHA);
    }

    public Battery(HardwareMap hardwareMap, double readIntervalMs, double alpha) {
        this.readIntervalMs = readIntervalMs;
        this.alpha = alpha;

        VoltageSensor sensor = null;
        if (hardwareMap != null && hardwareMap.voltageSensor != null) {
            for (VoltageSensor vs : hardwareMap.voltageSensor) {
                double v = vs.getVoltage();
                if (v > 0) {
                    sensor = vs;
                    this.filteredVoltage = v;
                    this.initialized = true;
                    break;
                }
            }
        }
        this.voltageSensor = sensor;
        this.timer.reset();
    }

    /**
     * Returns the low-pass filtered battery voltage in Volts.
     * Hardware read is throttled to at most every 200 ms.
     */
    public double volts() {
        if (voltageSensor != null && timer.milliseconds() >= readIntervalMs) {
            double raw = voltageSensor.getVoltage();
            timer.reset();
            if (raw > 0) {
                if (!initialized) {
                    filteredVoltage = raw;
                    initialized = true;
                } else {
                    filteredVoltage = alpha * raw + (1.0 - alpha) * filteredVoltage;
                }
            }
        }
        return filteredVoltage;
    }

    /** Alias for volts(). */
    public double getVoltage() {
        return volts();
    }
}
