package org.firstinspires.ftc.teamcode.sandboxes.Dennis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="U - Motor Spin Test", group="Linear Opmode")
public class MotorSpinTest extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lb");
        rightRear = hardwareMap.get(DcMotorEx.class, "rb");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        leftFront.setPower(0.3);
        leftRear.setPower(0.3);
        rightRear.setPower(0.3);
        rightFront.setPower(0.3);

        while(!isStopRequested()) {
            telemetry.addData("Left Front", leftFront.getPower());
            telemetry.addData("Left Rear", leftRear.getPower());
            telemetry.addData("Right Rear", rightRear.getPower());
            telemetry.addData("Right Front", rightFront.getPower());
            telemetry.update();
        }
    }

}
