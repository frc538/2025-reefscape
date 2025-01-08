// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class SwerveUtils {
    public static double stepTowards(double current, double target, double stepSize) {
        if (Math.abs(current - target) <= stepSize) {
            return target;
        } else if (target < current) {
            return current - stepSize;
        } else {
            return current + stepSize;
        }
    }

    public static double stepTowardsCircular(double current, double target, double stepSize) {
        current = wrapAngle(current);
        target = wrapAngle(target);

        double stepDirection = Math.signum(target - current);
        double difference = Math.abs(target - current);

        if (difference <= stepSize)
            return target;

        if (difference > Math.PI) {
            if (current + 2 * Math.PI - target < stepSize || target + 2 * Math.PI - current < stepSize) {
                return target;

            } else {
                return wrapAngle(current - stepDirection * stepSize);
            }
        } else {
            return current + stepDirection * stepSize;
        }
    }

    public static double wrapAngle(double angle) {
        final double twoPi = 2 * Math.PI;

        if (angle == twoPi)
            return 0;
        if (angle > twoPi)
            return angle - twoPi * Math.floor(angle / twoPi);
        if (angle < 0)
            return angle + twoPi * (Math.floor((-angle) / twoPi) + 1);
        return angle;
    }

    public static double angleDifference(double angleA, double angleB){
        double difference = Math.abs(angleA -angleB);
        return difference > Math.PI ? (2*Math.PI) - difference : difference;
    }
}
