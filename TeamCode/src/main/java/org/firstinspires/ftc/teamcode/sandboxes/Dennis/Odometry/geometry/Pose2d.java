package org.firstinspires.ftc.teamcode.sandboxes.Dennis.Odometry.geometry;

public class Pose2d {

    // geo
    double x;
    double y;
    double heading;

    // constructor
    public Pose2d(double x, double y, double heading) {
        this.x  = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d plus(Pose2d other) {
        return new Pose2d(x + other.x, y + other.y, heading + other.heading);
    }

    public Pose2d minus(Pose2d other) {
        return new Pose2d(x - other.x, y - other.y, heading - other.heading);
    }

    public Pose2d times(double scalar) {
        return new Pose2d(x * scalar, y * scalar, heading * scalar);
    }

    public Pose2d div(double scalar) {
        return new Pose2d(x / scalar, y / scalar, heading / scalar);
    }

    public String toString() {
        return String.format("(%.3f, %.3f, %.3f°)", x, y, Math.toDegrees(heading));
    }

}
