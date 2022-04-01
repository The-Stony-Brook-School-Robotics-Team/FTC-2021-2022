package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.sbs.bears.robotframework.controllers.RoadRunnerController;

public class AutomaticSlide {
    public static Pose2d blueShippingHub = new Pose2d(-12, 24, 0);
    public static int calculateSlidePosition(Pose2d pose){
        double distance = RoadRunnerController.distanceTwoPoints(pose, blueShippingHub);
        return (int) (28.6087*distance);
    }
}
