package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.sbs.bears.robotframework.controllers.RoadRunnerController;
@Config
public class AutomaticSlide {
    public static Pose2d blueShippingHub = new Pose2d(-12, 24, 0);
    public static double slideOffset = 150;
    public static double servoOffsetClose = -.05;
    public static double servoOffsetFar = -.4;
    public static int calculateSlidePosition(Pose2d pose){
        double distance = RoadRunnerController.distanceTwoPoints(pose, blueShippingHub);
        Log.d("AutomaticSlide","Distance is " + distance + ", extending to " + (int) (29.82542209*distance));
        return (int) ((29.82542209*distance-slideOffset < 0) ? 0 : (29.82542209*distance-slideOffset));
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
        double servoPos = 0.0395*Math.toDegrees(angle) - ((distance > 70) ? servoOffsetClose : servoOffsetFar);
        Log.d("AutomaticSlide","Distance is " + distance + ", lifting to angle " + Math.toDegrees(angle) + " servo Pos 0.0395*" + Math.toDegrees(angle) + "-" + servoOffsetClose + "=" + servoPos);
        return servoPos;
    }

}
