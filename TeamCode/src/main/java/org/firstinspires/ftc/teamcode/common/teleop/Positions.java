package org.firstinspires.ftc.teamcode.common.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Positions {

    /**
     * TeleOp Related Positions
     */
    public static Pose2d autonStartPose = new Pose2d(14, 65.5,0);

    /**
     * Deposit Positions
     */
    public static Pose2d blueDepositPosition = new Pose2d(5.58, 64.47, -Math.toRadians(57));

    /**
     * Warehouse Positions
     */
    public static Pose2d blueWarehouseResetPosition = new Pose2d(14,80,0);
    public static Pose2d blueWarehouseParkPosition = new Pose2d(60,80,0);
}
