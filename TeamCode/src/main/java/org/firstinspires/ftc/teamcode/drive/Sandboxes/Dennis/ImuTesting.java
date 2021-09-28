package org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.annotations.Testing;

import static org.firstinspires.ftc.teamcode.Robot.imu;

@Testing
@TeleOp(name="ByteLock IMU", group="drive")
public class ImuTesting extends OpMode {

    // Orientation
    Orientation currentOrientation;

    // ftc dash
    FtcDashboard dashboard;
    public static double AMPLITUDE = 10;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.5;

    @Override
    public void init()
    {
        Robot robot = new Robot(hardwareMap, telemetry);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    @Override
    public void init_loop() {
        currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("Calibration Status", imu.getCalibrationStatus());
        telemetry.addData("System Status", imu.getSystemStatus());
        telemetry.addLine();
        telemetry.addData("heading", AMPLITUDE * Math.sin(2 * Math.PI * FREQUENCY * currentOrientation.thirdAngle + Math.toRadians(PHASE)));
        telemetry.update();
    }

    @Override
    public void loop()
    {
        currentOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("Angular Orientation", currentOrientation);
        telemetry.addLine();
        telemetry.addData("Axes Order", currentOrientation.axesOrder);
        telemetry.addData("X", currentOrientation.firstAngle);
        telemetry.addData("Y", currentOrientation.secondAngle);
        telemetry.addData("Z", currentOrientation.thirdAngle);
        telemetry.addLine();
        telemetry.addLine("More Information");
        telemetry.addData("Temperature", imu.getTemperature().temperature);
        telemetry.addLine();
        telemetry.addLine("Rotation Rates");
        telemetry.addData("Xr", imu.getAngularVelocity().xRotationRate);
        telemetry.addData("Yr", imu.getAngularVelocity().yRotationRate);
        telemetry.addData("Zr", imu.getAngularVelocity().zRotationRate);
        telemetry.update();
    }

}
