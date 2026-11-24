package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.TreeSet;

/**
 * Utility for 2D Bilinear Interpolation of shooter hood angle (degrees) based on target distance (meters)
 * AND actual low-pass filtered flywheel velocity (ticks/sec).
 */
@SuppressWarnings("unused")
public class ShooterLookupTable {

    /**
     * 2D Sample Data Point representing (Distance, Flywheel Velocity, Hood Angle).
     */
    public static class Shooter2DPoint {
        private final double distance;          // Distance in meters
        private final double flywheelVelocity;  // Flywheel velocity in TPS
        private final double hoodAngleDegrees;  // Hood angle in physical degrees

        public Shooter2DPoint(double distance, double flywheelVelocity, double hoodAngleDegrees) {
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

    /**
     * Encapsulates target shooter state including nominal flywheel velocity and 2D-interpolated dynamic hood angle.
     */
    public static class ShooterTargetState {
        private final double distance;
        private final double nominalFlywheelVelocity;
        private final double nominalHoodAngleDegrees;
        private final double dynamicHoodAngleDegrees;

        public ShooterTargetState(double distance, double nominalFlywheelVelocity,
                                  double nominalHoodAngleDegrees, double dynamicHoodAngleDegrees) {
            this.distance = distance;
            this.nominalFlywheelVelocity = nominalFlywheelVelocity;
            this.nominalHoodAngleDegrees = nominalHoodAngleDegrees;
            this.dynamicHoodAngleDegrees = dynamicHoodAngleDegrees;
        }

        public ShooterTargetState(double distance, double nominalFlywheelVelocity, double nominalHoodAngleDegrees) {
            this(distance, nominalFlywheelVelocity, nominalHoodAngleDegrees, nominalHoodAngleDegrees);
        }

        public double getDistance() {
            return distance;
        }

        /** Returns target nominal flywheel velocity (TPS). */
        public double getFlywheelVelocity() {
            return nominalFlywheelVelocity;
        }

        public double getNominalFlywheelVelocity() {
            return nominalFlywheelVelocity;
        }

        /** Returns dynamic 2D-interpolated hood angle (degrees). */
        public double getHoodAngleDegrees() {
            return dynamicHoodAngleDegrees;
        }

        public double getNominalHoodAngleDegrees() {
            return nominalHoodAngleDegrees;
        }

        public double getDynamicHoodAngleDegrees() {
            return dynamicHoodAngleDegrees;
        }

        @NonNull
        @Override
        public String toString() {
            return String.format(Locale.US, "Dist: %.2fm, TargetVel: %.0f tps, Hood(Dyn/Nom): %.1f° / %.1f°",
                    distance, nominalFlywheelVelocity, dynamicHoodAngleDegrees, nominalHoodAngleDegrees);
        }
    }

    private final List<Shooter2DPoint> visionPoints2D = new ArrayList<>();
    private final List<Shooter2DPoint> odometryPoints2D = new ArrayList<>();

    public ShooterLookupTable() {
        // Empirical Vision Calibration Data Points from Master (Duplicated across 3 velocity tiers: v-150, v-75, v_nominal)
        addVisionPoint2D(0.36, 1200.0, 260.0);
        addVisionPoint2D(0.36, 1275.0, 260.0);
        addVisionPoint2D(0.36, 1350.0, 260.0);

        addVisionPoint2D(0.44, 1200.0, 260.0);
        addVisionPoint2D(0.44, 1275.0, 260.0);
        addVisionPoint2D(0.44, 1350.0, 260.0);

        addVisionPoint2D(0.50, 1200.0, 250.0);
        addVisionPoint2D(0.50, 1275.0, 250.0);
        addVisionPoint2D(0.50, 1350.0, 250.0);

        addVisionPoint2D(0.60, 1200.0, 250.0);
        addVisionPoint2D(0.60, 1275.0, 250.0);
        addVisionPoint2D(0.60, 1350.0, 250.0);

        addVisionPoint2D(0.70, 1250.0, 250.0);
        addVisionPoint2D(0.70, 1325.0, 250.0);
        addVisionPoint2D(0.70, 1400.0, 250.0);

        addVisionPoint2D(0.80, 1300.0, 250.0);
        addVisionPoint2D(0.80, 1375.0, 250.0);
        addVisionPoint2D(0.80, 1450.0, 250.0);

        addVisionPoint2D(0.90, 1300.0, 250.0);
        addVisionPoint2D(0.90, 1375.0, 250.0);
        addVisionPoint2D(0.90, 1450.0, 250.0);

        addVisionPoint2D(1.00, 1350.0, 250.0);
        addVisionPoint2D(1.00, 1425.0, 250.0);
        addVisionPoint2D(1.00, 1500.0, 250.0);

        addVisionPoint2D(1.10, 1400.0, 245.0);
        addVisionPoint2D(1.10, 1475.0, 245.0);
        addVisionPoint2D(1.10, 1550.0, 245.0);

        addVisionPoint2D(1.20, 1450.0, 245.0);
        addVisionPoint2D(1.20, 1525.0, 245.0);
        addVisionPoint2D(1.20, 1600.0, 245.0);

        addVisionPoint2D(1.30, 1450.0, 245.0);
        addVisionPoint2D(1.30, 1525.0, 245.0);
        addVisionPoint2D(1.30, 1600.0, 245.0);

        // Empirical Odometry Calibration Data Points from Master (Duplicated across 3 velocity tiers)
        addOdometryPoint2D(0.55, 1150.0, 270.0);
        addOdometryPoint2D(0.55, 1225.0, 270.0);
        addOdometryPoint2D(0.55, 1300.0, 270.0);

        addOdometryPoint2D(0.60, 1200.0, 250.0);
        addOdometryPoint2D(0.60, 1275.0, 250.0);
        addOdometryPoint2D(0.60, 1350.0, 250.0);

        addOdometryPoint2D(0.70, 1200.0, 240.0);
        addOdometryPoint2D(0.70, 1275.0, 240.0);
        addOdometryPoint2D(0.70, 1350.0, 240.0);

        addOdometryPoint2D(0.80, 1200.0, 230.0);
        addOdometryPoint2D(0.80, 1275.0, 230.0);
        addOdometryPoint2D(0.80, 1350.0, 230.0);

        addOdometryPoint2D(0.90, 1250.0, 210.0);
        addOdometryPoint2D(0.90, 1325.0, 210.0);
        addOdometryPoint2D(0.90, 1400.0, 210.0);

        addOdometryPoint2D(1.00, 1250.0, 190.0);
        addOdometryPoint2D(1.00, 1325.0, 190.0);
        addOdometryPoint2D(1.00, 1400.0, 190.0);

        addOdometryPoint2D(1.10, 1275.0, 180.0);
        addOdometryPoint2D(1.10, 1350.0, 180.0);
        addOdometryPoint2D(1.10, 1425.0, 180.0);

        addOdometryPoint2D(1.20, 1275.0, 180.0);
        addOdometryPoint2D(1.20, 1350.0, 180.0);
        addOdometryPoint2D(1.20, 1425.0, 180.0);

        addOdometryPoint2D(1.30, 1300.0, 185.0);
        addOdometryPoint2D(1.30, 1375.0, 185.0);
        addOdometryPoint2D(1.30, 1450.0, 185.0);

        addOdometryPoint2D(1.40, 1325.0, 175.0);
        addOdometryPoint2D(1.40, 1400.0, 175.0);
        addOdometryPoint2D(1.40, 1475.0, 175.0);
    }

    /** Adds a 2D data point to the vision map. */
    public void addVisionPoint2D(double distance, double flywheelVelocity, double hoodAngleDegrees) {
        visionPoints2D.add(new Shooter2DPoint(distance, flywheelVelocity, hoodAngleDegrees));
    }

    /** Legacy helper method for adding vision calibration point. */
    public void addPoint(double distance, double flywheelVelocity, double hoodAngleDegrees) {
        addVisionPoint2D(distance, flywheelVelocity, hoodAngleDegrees);
    }

    /** Adds a 2D data point to the odometry map. */
    public void addOdometryPoint2D(double distance, double flywheelVelocity, double hoodAngleDegrees) {
        odometryPoints2D.add(new Shooter2DPoint(distance, flywheelVelocity, hoodAngleDegrees));
    }

    /**
     * Primary 2D Bilinear Interpolation method. Given target distance (meters) and current flywheel velocity (TPS),
     * computes nominal flywheel velocity setpoint and 2D-interpolated dynamic hood angle.
     */
    public ShooterTargetState get2DInterpolatedState(double distance, double currentVelocity, AimSourceSelector.AimSource source) {
        List<Shooter2DPoint> activePoints = (source != null && source.isOdometry()) ? odometryPoints2D : visionPoints2D;
        if (activePoints.isEmpty()) {
            return new ShooterTargetState(distance, 0.0, 180.0, 180.0);
        }

        // 1. Get unique sorted distances available in dataset
        TreeSet<Double> distanceSet = new TreeSet<>();
        for (Shooter2DPoint pt : activePoints) {
            distanceSet.add(pt.getDistance());
        }

        List<Double> distances = new ArrayList<>(distanceSet);
        if (distances.isEmpty()) {
            return new ShooterTargetState(distance, 0.0, 180.0, 180.0);
        }

        // Clamp target distance to available range bounds
        double clampedDist = Math.max(distances.get(0), Math.min(distances.get(distances.size() - 1), distance));

        // Find bounding distance interval [d1, d2]
        double d1 = distances.get(0);
        double d2 = distances.get(distances.size() - 1);

        for (int i = 0; i < distances.size() - 1; i++) {
            if (clampedDist >= distances.get(i) && clampedDist <= distances.get(i + 1)) {
                d1 = distances.get(i);
                d2 = distances.get(i + 1);
                break;
            }
        }

        // 2. Interpolate 1D nominal target state at distance d1 and d2
        ShooterTargetState nominalD1 = getNominalStateAtExactDistance(activePoints, d1);
        ShooterTargetState nominalD2 = getNominalStateAtExactDistance(activePoints, d2);

        double tDist = (Math.abs(d2 - d1) < 1e-6) ? 0.0 : (clampedDist - d1) / (d2 - d1);
        double nominalVel = nominalD1.getNominalFlywheelVelocity() + tDist * (nominalD2.getNominalFlywheelVelocity() - nominalD1.getNominalFlywheelVelocity());
        double nominalHood = nominalD1.getNominalHoodAngleDegrees() + tDist * (nominalD2.getNominalHoodAngleDegrees() - nominalD1.getNominalHoodAngleDegrees());

        // 3. Interpolate 2D Hood Angle for current velocity at d1 and d2
        double hoodAtD1 = interpolateHoodAtDistance(activePoints, d1, currentVelocity);
        double hoodAtD2 = interpolateHoodAtDistance(activePoints, d2, currentVelocity);

        double dynamicHood = hoodAtD1 + tDist * (hoodAtD2 - hoodAtD1);

        return new ShooterTargetState(clampedDist, nominalVel, nominalHood, dynamicHood);
    }

    /** Returns interpolated 2D state for vision mode. */
    public ShooterTargetState getVisionInterpolatedState(double distance, double currentVelocity) {
        return get2DInterpolatedState(distance, currentVelocity, AimSourceSelector.AimSource.VISION);
    }

    /** Returns interpolated 2D state for odometry mode. */
    public ShooterTargetState getOdometryInterpolatedState(double distance, double currentVelocity) {
        return get2DInterpolatedState(distance, currentVelocity, AimSourceSelector.AimSource.ODOMETRY);
    }

    /** Legacy 1D interpolation compatibility wrappers (uses nominal velocity as current velocity). */
    public ShooterTargetState getVisionInterpolatedState(double distance) {
        ShooterTargetState nominal = getNominalStateAtDistance(visionPoints2D, distance);
        return get2DInterpolatedState(distance, nominal.getNominalFlywheelVelocity(), AimSourceSelector.AimSource.VISION);
    }

    public ShooterTargetState getOdometryInterpolatedState(double distance) {
        ShooterTargetState nominal = getNominalStateAtDistance(odometryPoints2D, distance);
        return get2DInterpolatedState(distance, nominal.getNominalFlywheelVelocity(), AimSourceSelector.AimSource.ODOMETRY);
    }

    public ShooterTargetState getInterpolatedState(double distance, AimSourceSelector.AimSource source) {
        if (source != null && source.isOdometry()) {
            return getOdometryInterpolatedState(distance);
        }
        return getVisionInterpolatedState(distance);
    }

    public ShooterTargetState getInterpolatedState(double distance) {
        return getVisionInterpolatedState(distance);
    }

    /** Helper: Bilinear interpolation across velocity points at a fixed distance d. */
    private double interpolateHoodAtDistance(List<Shooter2DPoint> points, double targetDistance, double velocity) {
        List<Shooter2DPoint> slice = new ArrayList<>();
        for (Shooter2DPoint pt : points) {
            if (Math.abs(pt.getDistance() - targetDistance) < 1e-4) {
                slice.add(pt);
            }
        }

        if (slice.isEmpty()) {
            return 180.0;
        }

        slice.sort(Comparator.comparingDouble(Shooter2DPoint::getFlywheelVelocity));

        if (slice.size() == 1 || velocity <= slice.get(0).getFlywheelVelocity()) {
            return slice.get(0).getHoodAngleDegrees();
        }

        if (velocity >= slice.get(slice.size() - 1).getFlywheelVelocity()) {
            return slice.get(slice.size() - 1).getHoodAngleDegrees();
        }

        for (int i = 0; i < slice.size() - 1; i++) {
            Shooter2DPoint p1 = slice.get(i);
            Shooter2DPoint p2 = slice.get(i + 1);

            if (velocity >= p1.getFlywheelVelocity() && velocity <= p2.getFlywheelVelocity()) {
                double range = p2.getFlywheelVelocity() - p1.getFlywheelVelocity();
                if (Math.abs(range) < 1e-6) {
                    return p1.getHoodAngleDegrees();
                }
                double u = (velocity - p1.getFlywheelVelocity()) / range;
                return p1.getHoodAngleDegrees() + u * (p2.getHoodAngleDegrees() - p1.getHoodAngleDegrees());
            }
        }

        return slice.get(slice.size() - 1).getHoodAngleDegrees();
    }

    private ShooterTargetState getNominalStateAtExactDistance(List<Shooter2DPoint> points, double targetDistance) {
        List<Shooter2DPoint> slice = new ArrayList<>();
        for (Shooter2DPoint pt : points) {
            if (Math.abs(pt.getDistance() - targetDistance) < 1e-4) {
                slice.add(pt);
            }
        }
        if (slice.isEmpty()) {
            return new ShooterTargetState(targetDistance, 1400.0, 180.0, 180.0);
        }
        // Highest velocity point in slice represents nominal state (provides maximum buffer for shooting)
        slice.sort(Comparator.comparingDouble(Shooter2DPoint::getFlywheelVelocity));
        Shooter2DPoint nominal = slice.get(slice.size() - 1);
        return new ShooterTargetState(targetDistance, nominal.getFlywheelVelocity(), nominal.getHoodAngleDegrees(), nominal.getHoodAngleDegrees());
    }

    private ShooterTargetState getNominalStateAtDistance(List<Shooter2DPoint> points, double distance) {
        TreeSet<Double> distanceSet = new TreeSet<>();
        for (Shooter2DPoint pt : points) {
            distanceSet.add(pt.getDistance());
        }
        List<Double> distances = new ArrayList<>(distanceSet);
        if (distances.isEmpty()) {
            return new ShooterTargetState(distance, 1400.0, 180.0, 180.0);
        }

        double clampedDist = Math.max(distances.get(0), Math.min(distances.get(distances.size() - 1), distance));

        double d1 = distances.get(0);
        double d2 = distances.get(distances.size() - 1);

        for (int i = 0; i < distances.size() - 1; i++) {
            if (clampedDist >= distances.get(i) && clampedDist <= distances.get(i + 1)) {
                d1 = distances.get(i);
                d2 = distances.get(i + 1);
                break;
            }
        }

        ShooterTargetState n1 = getNominalStateAtExactDistance(points, d1);
        ShooterTargetState n2 = getNominalStateAtExactDistance(points, d2);

        double t = (Math.abs(d2 - d1) < 1e-6) ? 0.0 : (clampedDist - d1) / (d2 - d1);
        double vel = n1.getNominalFlywheelVelocity() + t * (n2.getNominalFlywheelVelocity() - n1.getNominalFlywheelVelocity());
        double hood = n1.getNominalHoodAngleDegrees() + t * (n2.getNominalHoodAngleDegrees() - n1.getNominalHoodAngleDegrees());

        return new ShooterTargetState(clampedDist, vel, hood, hood);
    }

    public List<Shooter2DPoint> getVisionPoints2D() {
        return Collections.unmodifiableList(new ArrayList<>(visionPoints2D));
    }

    public List<Shooter2DPoint> getOdometryPoints2D() {
        return Collections.unmodifiableList(new ArrayList<>(odometryPoints2D));
    }
}
