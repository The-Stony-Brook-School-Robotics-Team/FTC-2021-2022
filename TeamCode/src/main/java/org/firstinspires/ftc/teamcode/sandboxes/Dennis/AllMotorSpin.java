package org.firstinspires.ftc.teamcode.sandboxes.Dennis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="U - Motor Spin Test", group="Linear Opmode")
public class AllMotorSpin extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "backodom");
        rightRear = hardwareMap.get(DcMotorEx.class, "leftodom");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightodom");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

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
