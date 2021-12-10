package org.firstinspires.ftc.teamcode.Sandboxes.William.Util;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Util {
    public static DcMotor RIGHT_FRONT_MOTOR;
    public static DcMotor RIGHT_BACK_MOTOR;
    public static DcMotor LEFT_FRONT_MOTOR;
    public static DcMotor LEFT_BACK_MOTOR;
    public static DcMotor LEFT_ODOMETRY_WHEEL;
    public static DcMotor RIGHT_ODOMETRY_WHEEL;
    public static DcMotor BACK_ODOMETRY_WHEEL;


    public void run(HardwareMap hardwareMap) {
        RIGHT_FRONT_MOTOR = hardwareMap.dcMotor.get(ControllerHelper.RIGHT_FRONT_MOTOR_NAME);
        RIGHT_BACK_MOTOR = hardwareMap.dcMotor.get(ControllerHelper.RIGHT_BACK_MOTOR_NAME);
        LEFT_FRONT_MOTOR = hardwareMap.dcMotor.get(ControllerHelper.LEFT_FRONT_MOTOR_NAME);
        LEFT_BACK_MOTOR = hardwareMap.dcMotor.get(ControllerHelper.LEFT_BACK_MOTOR_NAME);

        LEFT_ODOMETRY_WHEEL = hardwareMap.dcMotor.get(ControllerHelper.LEFT_ODOMETRY_NAME);
        RIGHT_ODOMETRY_WHEEL = hardwareMap.dcMotor.get(ControllerHelper.RIGHT_ODOMETRY_NAME);
        BACK_ODOMETRY_WHEEL = hardwareMap.dcMotor.get(ControllerHelper.BACK_ODOMETRY_NAME);
    }
}
