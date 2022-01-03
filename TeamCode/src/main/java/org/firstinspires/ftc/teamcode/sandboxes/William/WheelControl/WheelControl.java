package org.firstinspires.ftc.teamcode.Sandboxes.William.WheelControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by William Tao on 10/18/2021.
 * <p>
 * This code is used to test the movement and time of the motor which
 * is designed to spin the wheel on the corner of the field.
 * <p>
 * There are three stages during the spinning, which are called Stage 1 and Stage 2.
 * Stage 1: accelerate.
 * Stage 2: Stop the wheel.
 * <p>
 * Game pad Controls:
 * Press DpadUp & DpadDown to control the time for FRICTION_CONSTANT.
 * Press X & B on the game pad to control the time for FIRST_STAGE_TIME.
 * Press DpadLeft & DpadRight to control the time for SECOND_STAGE_TIME.
 * <p>
 * Press A to start a spin.
 * <p>
 * --------With MAGICAL_CONSTANT--------
 */

@TeleOp(name = "Wheel Control (Final)", group = "WC")
public class WheelControl extends OpMode {

    private boolean isPressingA = false;
    private boolean isPressingB = false;
    private boolean isPressingX = false;
    private boolean isPressingY = false;
    private boolean isPressingDpadUp = false;
    private boolean isPressingDpadDown = false;
    private boolean hasStarted = false;

    private boolean NEED_EMERGENCY_STOP = false;

    DcMotor wheelMover;
    private double FIRST_STAGE_TIME;
    private double SECOND_STAGE_TIME;

    private double MAGICAL_CONSTANT = 0.44;
    private double FIRST_STAGE_TIME_INTERVAL = 1.3;
    private final double SECOND_STAGE_TIME_INTERVAL = 0.1;

    private double timer;
    private double runTime;

    @Override
    public void init() {
        wheelMover = hardwareMap.get(DcMotor.class, "duck");
        wheelMover.setDirection(DcMotorSimple.Direction.REVERSE);
        initializeVariables();
    }

    @Override
    public void loop() {
        //Print data to the phone.
        telemetry.update();
        telemetry.addData("FIRST_STAGE_TIME_INTERVAL", "%.3f", FIRST_STAGE_TIME_INTERVAL);
        telemetry.addData("MAGICAL_CONSTANT", "--%.3f--", MAGICAL_CONSTANT);

        //Detect keys on the game pad.
        checkKeyA();        //Start spinning.
        checkKeyB();        //Check if need emergency Stop.
        checkKeyX();        //Add 0.01 second to Stage 2 time.
        checkKeyY();        //Subtract 0.01 second to Stage 2 time.
        checkDpadUp();      //Add 0.01 second to Stage 1 time.
        checkDpadDown();    //Subtract 0.01 second to Stage 1 time.

        enableEmergencyStop();

        //Control the movement of the wheel.
        updateMotorSpeed();
    }

    private void updateMotorSpeed() {
        if (hasStarted) {
            if (NEED_EMERGENCY_STOP) {
                wheelMover.setPower(0);
                hasStarted = false;
                NEED_EMERGENCY_STOP = false;
            }

            updateRunTime();
            if (runTime >= 0 && runTime < FIRST_STAGE_TIME) {
                //First Stage
                wheelMover.setPower(getFirstStageMotorSpeed(runTime));
            } else if (runTime >= FIRST_STAGE_TIME && runTime < SECOND_STAGE_TIME) {   //Third Stage
                wheelMover.setPower(-1);
            } else {
                //Ending
                wheelMover.setPower(0);
                hasStarted = false;
            }
        } else {
            initializeVariables();
        }
    }

    private void updateRunTime() {
        runTime = getRuntime() - timer;
    }

    private void initializeVariables() {
        FIRST_STAGE_TIME = FIRST_STAGE_TIME_INTERVAL;
        SECOND_STAGE_TIME = FIRST_STAGE_TIME + SECOND_STAGE_TIME_INTERVAL;
        timer = getRuntime();
    }

    /**
     * Control the motor speed for the first stage.
     *
     * @return The speed of the real-time target speed of the motor.
     */
    private double getFirstStageMotorSpeed() {
        return MAGICAL_CONSTANT * runTime;
    }

    private double getFirstStageMotorSpeed(double runTime) {
        return MAGICAL_CONSTANT * runTime;
    }

    private void enableEmergencyStop() {
        if (NEED_EMERGENCY_STOP) {
            NEED_EMERGENCY_STOP = false;
        }
    }

    private void checkKeyA() {
        if (gamepad1.a && !isPressingA) {
            isPressingA = true;
        } else if (!gamepad1.a && isPressingA) {
            if (!hasStarted) {
                hasStarted = true;
            } else {
                NEED_EMERGENCY_STOP = true;
            }
            isPressingA = false;
        }
    }

    private void checkKeyB() {
        if (gamepad1.b && !isPressingB) {
            isPressingB = true;
        } else if (!gamepad1.b && isPressingB) {
            NEED_EMERGENCY_STOP = true;
        }
        isPressingB = false;
    }

    private void checkKeyX() {
        if (gamepad1.x && !isPressingX) {
            isPressingX = true;
        } else if (!gamepad1.x && isPressingX) {
            MAGICAL_CONSTANT += 0.002;
            isPressingX = false;
        }
    }

    private void checkKeyY() {
        if (gamepad1.y && !isPressingY) {
            isPressingY = true;
        } else if (!gamepad1.y && isPressingY) {
            MAGICAL_CONSTANT -= 0.002;
            isPressingY = false;
        }
    }

    private void checkDpadUp() {
        if (gamepad1.dpad_up && !isPressingDpadUp) {
            isPressingDpadUp = true;
        } else if (!gamepad1.dpad_up && isPressingDpadUp) {
            FIRST_STAGE_TIME_INTERVAL += 0.005;
            isPressingDpadUp = false;
        }
    }

    private void checkDpadDown() {
        if (gamepad1.dpad_down && !isPressingDpadDown) {
            isPressingDpadDown = true;
        } else if (!gamepad1.dpad_down && isPressingDpadDown) {
            FIRST_STAGE_TIME_INTERVAL -= 0.005;
            isPressingDpadDown = false;
        }
    }
}