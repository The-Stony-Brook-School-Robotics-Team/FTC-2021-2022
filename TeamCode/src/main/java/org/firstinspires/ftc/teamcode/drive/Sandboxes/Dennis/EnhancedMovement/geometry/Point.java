package org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.geometry;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.annotations.Testing;
import org.jetbrains.annotations.NotNull;
import java.lang.Math;

// Public Class Point
public class Point {

    // Doubles to present x and y on a coordinate plane (Q1)
    public double x;
    public double y;
    public float heading;
    // "point" setup

    /**
     * Constructor for a point
     * @param x is the x position
     * @param y is the y position
     * @param heading is the robot heading
     */
    public Point(@NotNull double x,@NotNull double y, float heading)
    {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Point(@NotNull Point other) {
        this(other.x, other.y, other.heading);
    }

    // Get The Distance Between Two Points

    /**
     * Lets us get the distance between two points
     * @param other is the other point
     * @return will return the distance as a double
     */
    public double distance(@NotNull Point other) {
        return Math.hypot(other.x - this.x, other.y - this.y);
    }

    // TODO: Adjust equation
    /**
     * Gets the angle between two points (calc)
     * @param other
     * @return
     */
    @Testing
    public double angleTo(@NotNull Point other) {
        return Math.atan2(other.y - this.y, other.x - this.x);
    }
}
