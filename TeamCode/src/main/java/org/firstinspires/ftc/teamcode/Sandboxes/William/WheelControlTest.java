package org.firstinspires.ftc.teamcode.Sandboxes.William;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by William Tao on 10/18/2021.
 *
 * This code is used to test the movement and time of the motor which
 * is designed to spin the wheel on the corner of the field.
 *
 * There are three stages during the spinning, which are called Stage 1, Stage 2 and Stage 3.
 * Press DpadUp & DpadDown to control the time for Stage1.
 * Press X & B on the game pad to control the time for Stage 2.
 * Press DpadLeft & DpadRight to control the time for Stage 3.
 *
 * Press A to start a spin.
 */

@TeleOp(name = "Wheel Control Test", group = "DWW")
public class WheelControlTest extends OpMode {

    private boolean isPressingA = false;
    private boolean isPressingX = false;
    private boolean isPressingB = false;
    private boolean isPressingDpadUp = false;
    private boolean isPressingDpadDown = false;
    private boolean isPressingDpadLeft = false;
    private boolean isPressingDpadRight = false;
    private boolean hasStarted = false;

    DcMotor wheelMover;
    private Double frictionConstant = 0.45;
    private double FIRST_STAGE_TIME = 0.46;
    private double SECOND_STAGE_TIME = 0.7;
    private double THIRD_STAGE_TIME = 0.71;

    private Double timer;

    @Override
    public void init() {
        wheelMover = hardwareMap.get(DcMotor.class, "m1");
    }

    @Override
    public void loop() {
        //Print data to the phone.
        telemetry.update();
        telemetry.addData("FIRST_STAGE_TIME", "%.3f", FIRST_STAGE_TIME);
        telemetry.addData("SECOND_STAGE_TIME", "%.3f", SECOND_STAGE_TIME);
        telemetry.addData("THIRD_STAGE_TIME", "%.3f", THIRD_STAGE_TIME);

        //Detect keys on the Game-Pad.
        PressingA();            //Start spinning.
        PressingX();            //Add 0.01 second to Stage 2 time.
        PressingB();            //Subtract 0.01 second to Stage 2 time.
        PressingDpadUp();       //Add 0.01 second to Stage 1 time.
        PressingDpadDown();     //Subtract 0.01 second to Stage 1 time.
        PressingDpadLeft();     //Add 0.005 second to Stage 3 time.
        PressingDpadRight();    //Subtract 0.005 second to Stage 3 time.

        //Control the movement of the wheel.
        MovementHelper();
    }

    private void MovementHelper(){
        if (hasStarted) {
            Double runTime = getRuntime() - timer;
            if (runTime < FIRST_STAGE_TIME) {
                wheelMover.setPower(frictionConstant);
            } else if (runTime >= FIRST_STAGE_TIME && runTime <= SECOND_STAGE_TIME) {
                wheelMover.setPower(1);
            } else if (runTime >= THIRD_STAGE_TIME) {
                wheelMover.setPower(0);
                hasStarted = false;
            } else {
                wheelMover.setPower(-1);
            }
        }
    }

    private void PressingA() {
        if (gamepad1.a && !isPressingA) {
            isPressingA = true;
        } else if (!gamepad1.a && isPressingA) {
            if (!hasStarted) {
                timer = getRuntime();
                hasStarted = true;
            }
            isPressingA = false;
        }
    }

    private void PressingX() {
        if (gamepad1.x && !isPressingX) {
            isPressingX = true;
        } else if (!gamepad1.x && isPressingX) {
            SECOND_STAGE_TIME += 0.01;
            isPressingX = false;
        }
    }

    private void PressingB() {
        if (gamepad1.b && !isPressingB) {
            isPressingB = true;
        } else if (!gamepad1.b && isPressingB) {
            SECOND_STAGE_TIME -= 0.01;
            isPressingB = false;
        }
    }

    private void PressingDpadUp() {
        if (gamepad1.dpad_up && !isPressingDpadUp) {
            isPressingDpadUp = true;
        } else if (!gamepad1.dpad_up && isPressingDpadUp) {
            FIRST_STAGE_TIME += 0.01;
            isPressingDpadUp = false;
        }
    }

    private void PressingDpadDown() {
        if (gamepad1.dpad_down && !isPressingDpadDown) {
            isPressingDpadDown = true;
        } else if (!gamepad1.dpad_down && isPressingDpadDown) {
            FIRST_STAGE_TIME -= 0.01;
            isPressingDpadDown = false;
        }
    }

    private void PressingDpadLeft() {
        if(gamepad1.dpad_left && !isPressingDpadLeft) {
            isPressingDpadLeft = true;
        } else if(!gamepad1.dpad_left && isPressingDpadLeft) {
            THIRD_STAGE_TIME += 0.005;
            isPressingDpadLeft = false;
        }
    }

    private void PressingDpadRight() {
        if(gamepad1.dpad_right && !isPressingDpadRight) {
            isPressingDpadRight = true;
        } else if(!gamepad1.dpad_right && isPressingDpadRight) {
            THIRD_STAGE_TIME -= 0.005;
            isPressingDpadRight = false;
        }
    }
}