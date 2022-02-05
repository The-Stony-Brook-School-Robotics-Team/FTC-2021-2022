package org.firstinspires.ftc.teamcode.util;

import android.graphics.Color;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.coyote.framework.core.geometry.Pose2d;
import com.coyote.framework.core.geometry.Vector2d;
import com.coyote.framework.core.path.Path;


import java.util.List;

/**
 * Set of helper functions for drawing Road Runner paths and trajectories on dashboard canvases.
 */
public class DashboardUtil {
    private static final double DEFAULT_RESOLUTION = 2.0; // distance units; presumed inches
    private static final double ROBOT_RADIUS = 6; // in

    public static void drawPoseHistory(Canvas canvas, List<Pose2d> poseHistory) {
        if(internalColorIndex < 5) {
            canvas.setStroke("RED");
        } else if(internalColorIndex < 9) {
            canvas.setStroke("ORANGE");
        } else if(internalColorIndex < 13) {
            canvas.setStroke("YELLOW");
        } else if(internalColorIndex < 17) {
            canvas.setStroke("GREEN");
        } else if(internalColorIndex < 21) {
            canvas.setStroke("CYAN");
        } else if(internalColorIndex < 25) {
            canvas.setStroke("BLUE");
        } else if(internalColorIndex < 29) {
            canvas.setStroke("PINK");
        } else if(internalColorIndex < 29) {
            canvas.setStroke("MAGENTA");
        }

        canvas.setFill("GREEN");

        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path, double resolution) {
        int samples = (int) Math.ceil(path.length() / resolution);
        double[] xPoints = new double[samples];
        double[] yPoints = new double[samples];
        double dx = path.length() / (samples - 1);
        for (int i = 0; i < samples; i++) {
            double displacement = i * dx;
            Pose2d pose = path.get(displacement);
            xPoints[i] = pose.getX();
            yPoints[i] = pose.getY();
        }
        canvas.strokePolyline(xPoints, yPoints);
    }

    public static void drawSampledPath(Canvas canvas, Path path) {
        drawSampledPath(canvas, path, DEFAULT_RESOLUTION);
    }


    private static int internalColorIndex = 0;
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        if(internalColorIndex > 29) {
            internalColorIndex = 1;
        }
        if(internalColorIndex < 5) {
            canvas.setStroke("RED");
        } else if(internalColorIndex < 9) {
            canvas.setStroke("ORANGE");
        } else if(internalColorIndex < 13) {
            canvas.setStroke("YELLOW");
        } else if(internalColorIndex < 17) {
            canvas.setStroke("GREEN");
        } else if(internalColorIndex < 21) {
            canvas.setStroke("CYAN");
        } else if(internalColorIndex < 25) {
            canvas.setStroke("BLUE");
        } else if(internalColorIndex < 29) {
            canvas.setStroke("PINK");
        } else if(internalColorIndex < 29) {
            canvas.setStroke("MAGENTA");
        }

        // Border
        canvas.strokeRect(-72, -72, 144, 144);
        // Square
        // canvas.strokeRect(pose.getX() - 6, pose.getY() - 6, 12, 12);
        // Circle
        // canvas.strokeCircle(pose.getX(), pose.getY(), ROBOT_RADIUS);

        Vector2d v = pose.headingVec().times(ROBOT_RADIUS);
        double centerX1 = pose.getX() + v.getX() / 2;
        double centerY1 = pose.getY() + v.getY() / 2;
        double centerX2 = pose.getX() + v.getX();
        double centerY2 = pose.getY() + v.getY();



        // Draw the center line
        canvas.strokeLine(centerX1, centerY1, centerX2, centerY2);

        double length = 12;
        double t = (Math.atan((centerY2 - pose.getY()) / (centerX2 - pose.getX()))) + (Math.PI / 4);
        double a = Math.sqrt(2) * Math.sqrt(Math.pow(centerX2 - pose.getX(), 2) + Math.pow(centerY2 - pose.getY(), 2));

        // Point A
        double topLeftX = (a * Math.cos(t)) + pose.getX();
        double topLeftY = (a * Math.sin(t)) + pose.getY();

        // Point B
        double bottomLeftX = (a * Math.cos((Math.PI / 2) + t)) + pose.getX();
        double bottomLeftY = (a * Math.sin((Math.PI / 2) + t)) + pose.getY();

        // Point C
        double bottomRightX = bottomLeftX + (length * Math.sin(t - (Math.PI / 4)));
        double bottomRightY = bottomLeftY + (length * Math.cos(t + (3 * Math.PI / 4)));

        // Point D
        double topRightX = topLeftX + (length * Math.sin(t - (Math.PI / 4)));
        double topRightY = topLeftY + (length * Math.cos(t + (3 * Math.PI / 4)));

        // Left
        canvas.strokeLine(topLeftX, topLeftY, bottomLeftX, bottomLeftY);
        // Bottom
        canvas.strokeLine(bottomLeftX, bottomLeftY, bottomRightX, bottomRightY);
        // Right
        canvas.strokeLine(bottomRightX, bottomRightY, topRightX, topRightY);
        // Top
        canvas.strokeLine(topLeftX, topLeftY, topRightX, topRightY);




        internalColorIndex++;
    }
}
