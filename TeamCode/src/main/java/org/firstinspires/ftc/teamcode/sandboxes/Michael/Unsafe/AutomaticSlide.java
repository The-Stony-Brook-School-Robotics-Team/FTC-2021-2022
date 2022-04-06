package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.sbs.bears.robotframework.controllers.RoadRunnerController;

public class AutomaticSlide {
    public static Pose2d blueShippingHub = new Pose2d(-12, 24, 0);
    public static int calculateSlidePosition(Pose2d pose){
        double distance = RoadRunnerController.distanceTwoPoints(pose, blueShippingHub);
        Log.d("AutomaticSlide","Distance is " + distance + ", extending to " + (int) (29.82542209*distance));
        return (int) (29.82542209*distance);
    }
    public static double calculateTurnNeeded(Pose2d pose){
        //return blueShippingHub.getHeading() - pose.getHeading();

        //double deltaX = Math.sqrt(Math.pow(pose.getX() + blueShippingHub.getX(), 2));
        //double deltaY = Math.sqrt(Math.pow(pose.getY() + blueShippingHub.getY(), 2));
        return Math.atan2(pose.getY() - blueShippingHub.getY(), pose.getX() - blueShippingHub.getX());
    }
    public static double calculateServoPosNeeded(Pose2d pose)
    {
        double distance = RoadRunnerController.distanceTwoPoints(pose, blueShippingHub);
        double angle = Math.atan(19.0/distance);
        double servoPos = 0.0395*Math.toDegrees(angle) - ((distance > 1600) ? 0 : 0.10);
        Log.d("AutomaticSlide","Distance is " + distance + ", lifting to angle " + angle + " servo Pos " + servoPos);
        return servoPos;
    }

}
