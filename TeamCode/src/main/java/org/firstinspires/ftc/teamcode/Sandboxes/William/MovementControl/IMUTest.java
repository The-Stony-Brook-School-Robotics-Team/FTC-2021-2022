package org.firstinspires.ftc.teamcode.Sandboxes.William.MovementControl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name = "IMUTest", group = "Calibration")
public class IMUTest extends OpMode {
    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    TelemetryPacket telemetryPacket;
    public static FtcDashboard dashboard;

    @Override
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetryPacket = new TelemetryPacket();

        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        telemetryPacket.put("Angle", getZAngle());
        dashboard.sendTelemetryPacket(telemetryPacket);

        telemetry.addData("Angle", getZAngle());
        telemetry.update();
    }

    private double getZAngle() {
        return (-180 * Math.PI / imu.getAngularOrientation().firstAngle);
    }
}
