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
    public Vector2d convertPose2d(@NotNull Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

}
