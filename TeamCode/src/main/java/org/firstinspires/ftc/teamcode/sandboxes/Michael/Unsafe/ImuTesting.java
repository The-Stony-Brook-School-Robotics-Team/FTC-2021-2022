package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


public class ImuTesting extends LinearOpMode {

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu");



        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {

        }

    }

}
