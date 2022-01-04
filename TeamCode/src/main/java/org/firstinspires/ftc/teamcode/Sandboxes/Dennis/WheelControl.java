package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

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

public class WheelControl {

    //MAGICAL_CONSTANT should be between 0.39 to 0.41. Because of the lack of enough torque, the wheel actually never achieve the ideal acceleration.
    private static double MAGICAL_CONSTANT = 0.39;

    private static double FIRST_STAGE_TIME;
    private static double FIRST_STAGE_TIME_INTERVAL = 1.43;
    private static double MAX_WHEEL_SPEED = 0;
    private static double timer;
    private static double runTime;

    private static DcMotor wheelMover;

    /**
     * Start the program for spinning the wheel.
     * @param wheelMover
     */
    public static void start(DcMotor wheelMover) {
        initializeEnvironment(wheelMover);

        if (wheelMover.getPower() > MAX_WHEEL_SPEED)
            MAX_WHEEL_SPEED = wheelMover.getPower();

        //Control the movement of the wheel.
        while (updateMotorSpeed() != 2) ;
    }

    private static void initializeEnvironment(DcMotor wheelMover) {
        wheelMover.setDirection(DcMotorSimple.Direction.REVERSE);
        initializeVariables();
        WheelControl.wheelMover = wheelMover;
    }

    private static int updateMotorSpeed() {
        updateRunTime();
        if (runTime >= 0 && runTime < FIRST_STAGE_TIME) {
            //First Stage
            wheelMover.setPower(getFirstStageMotorSpeed(runTime));
            return 1;
        } else {
            //Ending
            wheelMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return 2;
        }
    }


    private static void updateRunTime() {
        runTime = getCurrentSystemSecond() - timer;
    }

    private static void initializeVariables() {
        FIRST_STAGE_TIME = FIRST_STAGE_TIME_INTERVAL;
        timer = getCurrentSystemSecond();
    }

    /**
     * Control the motor speed for the first stage.
     *
     * @return The speed of the real-time target speed of the motor.
     */
    private static double getFirstStageMotorSpeed(double runTime) {
        return MAGICAL_CONSTANT * runTime;
    }

    private static double getCurrentSystemSecond() {
        return System.currentTimeMillis() / 1000.0;
    }
}