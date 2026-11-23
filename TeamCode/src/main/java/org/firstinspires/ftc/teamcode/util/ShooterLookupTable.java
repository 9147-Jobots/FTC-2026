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

    /**
     * 8-point lookup array for Odometry: { distance (meters), rpm (flywheel velocity), hood angle (degrees) }.
     * Pre-populated with baseline calibration curve for immediate deployment; software engineers can tune below.
     */
    private final double[][] odometryLookupArray = new double[][] {
        {0.55, 1300.0, 270.0},
        {0.60, 1350.0, 250.0},
        {0.70, 1350.0, 240.0},
        {0.80, 1350.0, 230.0},
        {0.90, 1400.0, 210.0},
        {1.00, 1400.0, 190.0},
        {1.10, 1425.0, 180.0},
    };


    public ShooterLookupTable() {
        // Empirical vision calibration data points (distance in meters, velocity in ticks/sec, hood angle in degrees)
        addPoint(0.36, 1350.0, 260.0);
        addPoint(0.44, 1350.0, 260.0);
        addPoint(0.50, 1350.0, 250.0);
        addPoint(0.60, 1350.0, 250.0);
        addPoint(0.70, 1400.0, 250.0);
        addPoint(0.80, 1450.0, 250.0);
        addPoint(0.90, 1450.0, 250.0);
        addPoint(1.00, 1500.0, 250.0);
        addPoint(1.10, 1550.0, 245.0);
        addPoint(1.20, 1600.0, 245.0);
        addPoint(1.30, 1600.0, 245.0);
    }

    /**
     * Returns the 8-point odometry lookup array.
     */
    public double[][] getOdometryArray() {
        return odometryLookupArray;
    }

    /**
     * Adds a data point to the vision lookup table and maintains ascending order by distance.
     */
    public void addPoint(double distance, double flywheelVelocity, double hoodAngleDegrees) {
        points.add(new ShooterTargetState(distance, flywheelVelocity, hoodAngleDegrees));
        points.sort(Comparator.comparingDouble(ShooterTargetState::getDistance));
    }

    /**
     * Returns the interpolated shooter target state for vision lookup table.
     */
    public ShooterTargetState getVisionInterpolatedState(double distance) {
        return getInterpolatedState(distance);
    }

    /**
     * Returns interpolated target state from the 8-point odometry array for a given horizontal distance.
     */
    public ShooterTargetState getOdometryInterpolatedState(double distance) {
        List<ShooterTargetState> activeOdometryPoints = new ArrayList<>();
        for (double[] point : odometryLookupArray) {
            if (point != null && point.length >= 3) {
                activeOdometryPoints.add(new ShooterTargetState(point[0], point[1], point[2]));
            }
        }
        activeOdometryPoints.sort(Comparator.comparingDouble(ShooterTargetState::getDistance));

        return interpolateList(activeOdometryPoints, distance);
    }

    /**
     * Selects between vision or odometry lookup tables using AimSource selector.
     */
    public ShooterTargetState getInterpolatedState(double distance, AimSourceSelector.AimSource source) {
        if (source != null && source.isOdometry()) {
            return getOdometryInterpolatedState(distance);
        }
        return getVisionInterpolatedState(distance);
    }

    /**
     * Returns the interpolated shooter target state for a given target distance (in meters) from vision points.
     * If distance is outside the minimum or maximum sample bounds, the result is clamped to the nearest bound.
     */
    public ShooterTargetState getInterpolatedState(double distance) {
        return interpolateList(points, distance);
    }

    private ShooterTargetState interpolateList(List<ShooterTargetState> samplePoints, double distance) {
        if (samplePoints.isEmpty()) {
            return new ShooterTargetState(distance, 0.0, 0.0);
        }

        if (samplePoints.size() == 1) {
            ShooterTargetState single = samplePoints.get(0);
            return new ShooterTargetState(distance, single.getFlywheelVelocity(), single.getHoodAngleDegrees());
        }

        // Clamp below lower bound
        if (distance <= samplePoints.get(0).getDistance()) {
            ShooterTargetState min = samplePoints.get(0);
            return new ShooterTargetState(distance, min.getFlywheelVelocity(), min.getHoodAngleDegrees());
        }

        // Clamp above upper bound
        if (distance >= samplePoints.get(samplePoints.size() - 1).getDistance()) {
            ShooterTargetState max = samplePoints.get(samplePoints.size() - 1);
            return new ShooterTargetState(distance, max.getFlywheelVelocity(), max.getHoodAngleDegrees());
        }

        // Find bounding interval P_i and P_{i+1}
        for (int i = 0; i < samplePoints.size() - 1; i++) {
            ShooterTargetState p1 = samplePoints.get(i);
            ShooterTargetState p2 = samplePoints.get(i + 1);

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
        ShooterTargetState last = samplePoints.get(samplePoints.size() - 1);
        return new ShooterTargetState(distance, last.getFlywheelVelocity(), last.getHoodAngleDegrees());
    }

    public List<ShooterTargetState> getPoints() {
        return Collections.unmodifiableList(new ArrayList<>(points));
    }
}
