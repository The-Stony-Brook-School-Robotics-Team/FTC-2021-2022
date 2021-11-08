package org.firstinspires.ftc.teamcode.Sandboxes.William.MovementControl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class IMUTest extends OpMode {
    BNO055IMU imu;
    String rightFrontMotorName = ControllerHelper.rightFrontMotorName;
    String rightBackMotorName = ControllerHelper.rightBackMotorName;
    String leftFrontMotorName = ControllerHelper.leftFrontMotorName;
    String leftBackMotorName = ControllerHelper.leftBackMotorName;
    String leftOdometryName = ControllerHelper.leftOdometryName;
    String rightOdometryName = ControllerHelper.rightOdometryName;
    String backOdometryName = ControllerHelper.backOdometryName;

    @Override
    public void init() {
        //Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    @Override
    public void loop() {
        telemetry.addData("angle", getZAngle());
    }

    private double getZAngle() {
        return (-imu.getAngularOrientation().firstAngle);
    }
}
