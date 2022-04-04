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
        return blueShippingHub.getHeading() - pose.getHeading();
    }

}
