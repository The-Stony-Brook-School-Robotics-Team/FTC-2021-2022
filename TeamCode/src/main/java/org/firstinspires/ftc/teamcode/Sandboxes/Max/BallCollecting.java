package org.firstinspires.ftc.teamcode.Sandboxes.Max;


import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class BallCollecting extends LinearOpMode {
    private DcMotor Motor;

    @Override
    public void runOpMode() throws InterruptedException{
        Motor = hardwareMap.get(DcMotor.class, "1");

        waitForStart();

        while(true){

            Motor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Power: ", Motor.getPower());
            telemetry.update();
        }

    }
}
