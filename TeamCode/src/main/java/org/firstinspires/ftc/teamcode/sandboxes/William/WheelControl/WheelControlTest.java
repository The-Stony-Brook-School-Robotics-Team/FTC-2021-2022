package org.firstinspires.ftc.teamcode.sandboxes.William.WheelControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by William Tao on 10/18/2021.
 * <p>
 * This code is used to test the movement and time of the motor which
 * is designed to spin the wheel on the corner of the field.
 * <p>
 * There are three stages during the spinning, which are called Stage 1, Stage 2 and Stage 3.
 * <p>
 * Press X & B to control the value of MAGICAL_CONSTANT.
 * Press DpadUp & DpadDown on the game pad to control the time for FIRST_STAGE_TIME_INTERVAL.
 * Press DpadLeft & DpadRight to control the time for SECOND_STAGE_TIME_INTERVAL.
 * Press A to start a spin.
 */
@TeleOp(name = "Wheel Control (Test)", group = "WC")
public class WheelControlTest extends OpMode {

    private boolean isPressingA = false;
    private boolean isPressingX = false;
    private boolean isPressingB = false;
    private boolean isPressingDpadUp = false;
    private boolean isPressingDpadDown = false;
    private boolean isPressingDpadLeft = false;
    private boolean isPressingDpadRight = false;
    private boolean hasStarted = false;

    private boolean emergencyStop = false;

    DcMotor wheelMover;
    private double FIRST_STAGE_TIME;
    private double SECOND_STAGE_TIME;
    private double THIRD_STAGE_TIME;

    private double MAGICAL_CONSTANT = 0.45;
    private double FIRST_STAGE_TIME_INTERVAL = 0.46;
    private double SECOND_STAGE_TIME_INTERVAL = 0.24;
    private final double THIRD_STAGE_TIME_INTERVAL = 0.025;

    private double timer;
    private double runTime;

    @Override
    public void init() {
        wheelMover = hardwareMap.get(DcMotor.class, "m1");
        WheelControlInitializer();
    }

    @Override
    public void loop() {
        //Print data to the phone.
        telemetry.update();
        telemetry.addData("FIRST_STAGE_TIME_INTERVAL", "%.3f", FIRST_STAGE_TIME_INTERVAL);
        telemetry.addData("SECOND_STAGE_TIME_INTERVAL", "%.3f", SECOND_STAGE_TIME_INTERVAL);
        telemetry.addData("THIRD_STAGE_TIME_INTERVAL", "%.3f", THIRD_STAGE_TIME_INTERVAL);
        telemetry.addData("MAGICAL_CONSTANT", "--%.3f--", MAGICAL_CONSTANT);

        //Detect keys on the game pad.
        PressingA();            //Start spinning.
        PressingX();            //Add 0.01 second to Stage 2 time.
        PressingB();            //Subtract 0.01 second to Stage 2 time.
        PressingDpadUp();       //Add 0.01 second to Stage 1 time.
        PressingDpadDown();     //Subtract 0.01 second to Stage 1 time.
        PressingDpadLeft();     //Add 0.005 second to Stage 3 time.
        PressingDpadRight();    //Subtract 0.005 second to Stage 3 time.

        EmergencyStop();

        //Control the movement of the wheel.
        MovementHelper();
    }

    private void MovementHelper() {
        if (hasStarted) {
            runTime = getRuntime() - timer;
            if (runTime >= 0 && runTime < FIRST_STAGE_TIME) {
                //First Stage
                wheelMover.setPower(GetFirstStageMotorSpeed());
            } else if (runTime >= FIRST_STAGE_TIME && runTime < SECOND_STAGE_TIME) {
                //Second Stage
                wheelMover.setPower(1);
            } else if (runTime >= SECOND_STAGE_TIME && runTime < THIRD_STAGE_TIME) {
                //Third Stage
                wheelMover.setPower(-1);
            } else {
                //Ending
                wheelMover.setPower(0);
                hasStarted = false;
            }
        } else {
            WheelControlInitializer();
        }
    }

    private void WheelControlInitializer() {
        double MAX_FIRST_STAGE_TIME_INTERVAL = 1 / Math.sqrt(MAGICAL_CONSTANT);
        if (FIRST_STAGE_TIME_INTERVAL > MAX_FIRST_STAGE_TIME_INTERVAL) {
            SECOND_STAGE_TIME_INTERVAL += FIRST_STAGE_TIME_INTERVAL - MAX_FIRST_STAGE_TIME_INTERVAL;
            FIRST_STAGE_TIME_INTERVAL = MAX_FIRST_STAGE_TIME_INTERVAL;
        }

        FIRST_STAGE_TIME = FIRST_STAGE_TIME_INTERVAL;
        SECOND_STAGE_TIME = FIRST_STAGE_TIME + SECOND_STAGE_TIME_INTERVAL;
        THIRD_STAGE_TIME = SECOND_STAGE_TIME + THIRD_STAGE_TIME_INTERVAL;
        timer = getRuntime();
    }

    /**
     * Control the motor speed for the first stage.
     *
     * @return The speed of the real-time target speed of the motor.
     */
    private double GetFirstStageMotorSpeed() {
        return MAGICAL_CONSTANT * runTime;
    }

    private void PressingA() {
        if (gamepad1.a && !isPressingA) {
            isPressingA = true;
        } else if (!gamepad1.a && isPressingA) {
            if (!hasStarted) {
                hasStarted = true;
            } else {
                emergencyStop = true;
            }
            isPressingA = false;
        }
    }

    private void EmergencyStop() {
        if (emergencyStop) {
            emergencyStop = false;
        }
    }

    private void PressingX() {
        if (gamepad1.x && !isPressingX) {
            isPressingX = true;
        } else if (!gamepad1.x && isPressingX) {
            MAGICAL_CONSTANT += 0.002;
            isPressingX = false;
        }
    }

    private void PressingB() {
        if (gamepad1.b && !isPressingB) {
            isPressingB = true;
        } else if (!gamepad1.b && isPressingB) {
            MAGICAL_CONSTANT -= 0.002;
            isPressingB = false;
        }
    }

    private void PressingDpadUp() {
        if (gamepad1.dpad_up && !isPressingDpadUp) {
            isPressingDpadUp = true;
        } else if (!gamepad1.dpad_up && isPressingDpadUp) {
            FIRST_STAGE_TIME_INTERVAL += 0.005;
            isPressingDpadUp = false;
        }
    }

    private void PressingDpadDown() {
        if (gamepad1.dpad_down && !isPressingDpadDown) {
            isPressingDpadDown = true;
        } else if (!gamepad1.dpad_down && isPressingDpadDown) {
            FIRST_STAGE_TIME_INTERVAL -= 0.005;
            isPressingDpadDown = false;
        }
    }

    private void PressingDpadLeft() {
        if (gamepad1.dpad_left && !isPressingDpadLeft) {
            isPressingDpadLeft = true;
        } else if (!gamepad1.dpad_left && isPressingDpadLeft) {
            SECOND_STAGE_TIME_INTERVAL += 0.005;
            isPressingDpadLeft = false;
        }
    }

    private void PressingDpadRight() {
        if (gamepad1.dpad_right && !isPressingDpadRight) {
            isPressingDpadRight = true;
        } else if (!gamepad1.dpad_right && isPressingDpadRight) {
            SECOND_STAGE_TIME_INTERVAL -= 0.005;
            isPressingDpadRight = false;
        }
    }
}