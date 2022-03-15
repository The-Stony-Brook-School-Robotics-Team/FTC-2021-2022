package org.firstinspires.ftc.teamcode.sandboxes.William.WheelControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name = "TestBreak", group = "-WC")
public class TestBreak extends OpMode {
    DcMotor wheelMover;
    double power = 0.2;

    private boolean isPressingDpadUp = false;
    private boolean isPressingDpadDown = false;

    private boolean isPressingX = false;
    private boolean isPressingY = false;
    private boolean isPressingB = false;

    private double breakTime = 1.0;

    private double timer;
    private double runTime;

    private boolean startStop = false;
    private boolean started = false;

    @Override
    public void init() {
        wheelMover = hardwareMap.get(DcMotor.class, "duck");
        wheelMover.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (startStop) {
            if (!started) {
                timer = getRuntime();
                started = true;
            }
            updateRunTime();
            if (runTime >= 0 && runTime <= breakTime) {
                wheelMover.setPower(-1);
                telemetry.addData("Status", "BREAKING");
            } else {
                wheelMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                telemetry.addData("Status", "RESTING");
            }
        } else {
            started = false;
            wheelMover.setPower(power);
        }

        telemetry.addData("Current Speed", "--%.4f--", wheelMover.getPower());
        telemetry.addData("Break Time", "--%.4f--", breakTime);

        checkDpadUp();
        checkDpadDown();
        checkKeyB();
        checkKeyX();
        checkKeyY();
    }

    private void updateRunTime() {
        runTime = getRuntime() - timer;
    }


    private void checkDpadUp() {
        if (gamepad1.dpad_up && !isPressingDpadUp) {
            isPressingDpadUp = true;
        } else if (!gamepad1.dpad_up && isPressingDpadUp) {
            power += 0.01;
            isPressingDpadUp = false;
        }
    }

    private void checkDpadDown() {
        if (gamepad1.dpad_down && !isPressingDpadDown) {
            isPressingDpadDown = true;
        } else if (!gamepad1.dpad_down && isPressingDpadDown) {
            power -= 0.01;
            isPressingDpadDown = false;
        }
    }

    private void checkKeyB() {
        if (gamepad1.b && !isPressingB) {
            isPressingB = true;
        } else if (!gamepad1.b && isPressingB) {
            startStop = !startStop;
        }
        isPressingB = false;
    }

    private void checkKeyX() {
        if (gamepad1.x && !isPressingX) {
            isPressingX = true;
        } else if (!gamepad1.x && isPressingX) {
            breakTime += 0.02;
            isPressingX = false;
        }
    }

    private void checkKeyY() {
        if (gamepad1.y && !isPressingY) {
            isPressingY = true;
        } else if (!gamepad1.y && isPressingY) {
            breakTime -= 0.02;
            isPressingY = false;
        }
    }

}
