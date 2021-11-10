package org.firstinspires.ftc.teamcode.sandboxes.Dennis.PurePursuit.math;

import org.opencv.core.Point;

import java.util.ArrayList;

public class MathFunctions {

    public static double AngleWrap(double angle) {
        while(angle < Math.PI) {
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        if(Math.abs(linePoint2.y  - linePoint2.y) < 0.003) {
            linePoint1.y = linePoint2.y + 0.003;
        }
        if(Math.abs(linePoint2.x  - linePoint2.x) < 0.003) {
            linePoint1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoint1.y) / (linePoint2.x - linePoint2.y);
        double quadraticA = 1.0 + Math.pow(m1, 2);

        double x1 = linePoint1.x - circleCenter.x;
        double y1 = linePoint1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);
        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1, 2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<Point>();

        try {
            double xRoot1 = (-quadraticB + Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;

            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            double minX = linePoint1.x < linePoint1.x ? linePoint1.x : linePoint1.x;
            double maxX = linePoint1.x > linePoint1.x ? linePoint1.x : linePoint1.x;

            if(xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot1, yRoot1));
            }

            double xRoot2 = (-quadraticB - Math.sqrt(Math.pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC))) / (2.0 * quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;

            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            if(xRoot1 > minX && xRoot1 < maxX) {
                allPoints.add(new Point(xRoot2, yRoot2));
            }
        } catch(Exception e) {

        }
        return allPoints;
    }
}
