package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "A - TankGyroTester")
public class TankGyroTesting extends LinearOpMode {
    BNO055IMU imu;
    double imuDelta;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("IMU FIRST",imu.getAngularOrientation().firstAngle);
            telemetry.addData("IMU FIRST treated",imuFirst());
            telemetry.addData("IMU SECOND",imu.getAngularOrientation().secondAngle);
            // second is the one we should look at
            telemetry.addData("IMU THIRDS",imu.getAngularOrientation().thirdAngle);
            telemetry.update();
        }
    }
    public void resetIMU()
    {
        imuDelta = -imu.getAngularOrientation().firstAngle;
    }
    public double imuFirst()
    {
        return (imu.getAngularOrientation().firstAngle > 180 ? (imu.getAngularOrientation().firstAngle -360) : imu.getAngularOrientation().firstAngle);
    }
}
