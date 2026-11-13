package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * Utility for 1D linear interpolation of shooter flywheel velocity (ticks/sec)
 * and hood angle (degrees) based on distance to target.
 */
@SuppressWarnings("unused")
public class ShooterLookupTable {

    public static class ShooterTargetState {
        private final double distance;
        private final double flywheelVelocity;
        private final double hoodAngleDegrees;

        public ShooterTargetState(double distance, double flywheelVelocity, double hoodAngleDegrees) {
            this.distance = distance;
            this.flywheelVelocity = flywheelVelocity;
            this.hoodAngleDegrees = hoodAngleDegrees;
        }

        public double getDistance() {
            return distance;
        }

        public double getFlywheelVelocity() {
            return flywheelVelocity;
        }

        public double getHoodAngleDegrees() {
            return hoodAngleDegrees;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.US, "Dist: %.2fm, Vel: %.0f tps, Hood: %.1f°",
                    distance, flywheelVelocity, hoodAngleDegrees);
        }
    }

    private final List<ShooterTargetState> points = new ArrayList<>();

    public ShooterLookupTable() {
        // Default calibration data points (distance in meters, velocity in ticks/sec, hood angle in degrees)
        addPoint(0.36, 1350, 260);
        addPoint(0.44, 1350, 260);
        addPoint(0.50,1350,250);
        addPoint(0.60,1350,250);
        addPoint(0.70,1400,250);
        addPoint(0.8,1450,250);
        addPoint(0.9, 1450, 250);
        addPoint(1.0, 1500, 250);
        addPoint(1.1, 1550, 245);
        addPoint(1.2, 1600, 245);
        addPoint(1.3, 1600, 245);
    }

    /**
     * Adds a data point to the lookup table and maintains ascending order by distance.
     */
    public void addPoint(double distance, double flywheelVelocity, double hoodAngleDegrees) {
        points.add(new ShooterTargetState(distance, flywheelVelocity, hoodAngleDegrees));
        points.sort(Comparator.comparingDouble(ShooterTargetState::getDistance));
    }

    /**
     * Returns the interpolated shooter target state for a given target distance (in meters).
     * If distance is outside the minimum or maximum sample bounds, the result is clamped to the nearest bound.
     */
    public ShooterTargetState getInterpolatedState(double distance) {
        if (points.isEmpty()) {
            return new ShooterTargetState(distance, 0.0, 0.0);
        }

        if (points.size() == 1) {
            ShooterTargetState single = points.get(0);
            return new ShooterTargetState(distance, single.getFlywheelVelocity(), single.getHoodAngleDegrees());
        }

        // Clamp below lower bound
        if (distance <= points.get(0).getDistance()) {
            ShooterTargetState min = points.get(0);
            return new ShooterTargetState(distance, min.getFlywheelVelocity(), min.getHoodAngleDegrees());
        }

        // Clamp above upper bound
        if (distance >= points.get(points.size() - 1).getDistance()) {
            ShooterTargetState max = points.get(points.size() - 1);
            return new ShooterTargetState(distance, max.getFlywheelVelocity(), max.getHoodAngleDegrees());
        }

        // Find bounding interval P_i and P_{i+1}
        for (int i = 0; i < points.size() - 1; i++) {
            ShooterTargetState p1 = points.get(i);
            ShooterTargetState p2 = points.get(i + 1);

            if (distance >= p1.getDistance() && distance <= p2.getDistance()) {
                double range = p2.getDistance() - p1.getDistance();
                if (Math.abs(range) < 1e-6) {
                    return new ShooterTargetState(distance, p1.getFlywheelVelocity(), p1.getHoodAngleDegrees());
                }

                double t = (distance - p1.getDistance()) / range;
                double interpolatedVelocity = p1.getFlywheelVelocity() + t * (p2.getFlywheelVelocity() - p1.getFlywheelVelocity());
                double interpolatedHoodAngle = p1.getHoodAngleDegrees() + t * (p2.getHoodAngleDegrees() - p1.getHoodAngleDegrees());

                return new ShooterTargetState(distance, interpolatedVelocity, interpolatedHoodAngle);
            }
        }

        // Fallback to last point
        ShooterTargetState last = points.get(points.size() - 1);
        return new ShooterTargetState(distance, last.getFlywheelVelocity(), last.getHoodAngleDegrees());
    }

    public List<ShooterTargetState> getPoints() {
        return Collections.unmodifiableList(new ArrayList<>(points));
    }
}
