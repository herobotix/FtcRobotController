package org.firstinspires.ftc.schaub.teleop;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
public class VisionTarget {
    public double heading; // DEGREES
    public double distance; // METERS

    public VisionTarget(double i_heading, double i_distance) {
        heading = i_heading;
        distance = i_distance;
    }
    public VisionTarget() {
        heading = 0.0;
        distance = 0.0;
    }

    public double getDistance() {
        return distance;
    }

    public double getHeading() {
        double result = heading;
        // Normalize headingDifference to be between -180 and 180 degrees
        if (result > 180) {
            result -= 360;
        } else if (result < -180) {
            result += 360;
        }
        return result;
    }
}
