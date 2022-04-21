package org.firstinspires.ftc.teamcode.sandboxes.Dennis.debug;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
public class EncoderCounter extends LinearOpMode {

    DcMotor left, right, center;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotor.class, "leftodom");
        right = hardwareMap.get(DcMotor.class, "rightodom");
        center = hardwareMap.get(DcMotor.class, "duck");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        center.setDirection(DcMotorSimple.Direction.REVERSE);

        while(!isStopRequested()) {
            telemetry.addData("left ticks: ", left.getCurrentPosition());
            telemetry.addData("right ticks: ", right.getCurrentPosition());
            telemetry.addData("center ticks: ", center.getCurrentPosition());
            telemetry.update();
        }

    }
}
