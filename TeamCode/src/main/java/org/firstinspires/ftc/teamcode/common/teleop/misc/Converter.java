package org.firstinspires.ftc.teamcode.common.teleop.misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.jetbrains.annotations.NotNull;

public class Converter {

    /**
     * So Cool Converteerererererererer
     * @param pose
     * @return
     */
    public static Vector2d convertPose2d(@NotNull Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

    /**
     * Inches to Ticks
     */
    public static double InchesToTicks(@NotNull double inches) {
        double lower = 2 * Math.PI * 0.785;
        double div = 145.1 / lower;
        return div * inches;
    }



}
