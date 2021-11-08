package org.firstinspires.ftc.teamcode.Sandboxes.William.MovementControl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IMUTest", group = "Calibration")
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        telemetry.addData("angle", getZAngle());
    }

    private double getZAngle() {
        return (-180 * Math.PI / imu.getAngularOrientation().firstAngle);
    }
}
