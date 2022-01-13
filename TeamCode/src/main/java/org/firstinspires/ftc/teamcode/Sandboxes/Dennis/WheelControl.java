package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.roadrunner.util.NanoClock;
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

    //MAGICAL_CONSTANT should be between 0.30 to 0.40. Because of the lack of enough torque, the wheel actually never achieve the ideal acceleration.
    private static double MAGICAL_CONSTANT = 0.3;

    private static double FIRST_STAGE_TIME;
    private static double FIRST_STAGE_TIME_INTERVAL = 1.5;
    private static double timer;
    private static double runTime;

    private static DcMotor wheelMover;

    //------------------------------------------------------------
    public static void spinWheel(DcMotor wheelMover) {          //
        initializeEnvironment(wheelMover);                      //
        while (updateMotorSpeed()) ;                            //
    }                                                           //
    //------------------------------------------------------------

    private static double getCurrentSystemSecond() {
        return NanoClock.system().seconds();
    }

    public static void initializeEnvironment(DcMotor wheelMover) {
        wheelMover.setDirection(DcMotorSimple.Direction.REVERSE);
        WheelControl.wheelMover = wheelMover;
        initializeVariables();
        initializeVariables();
    }

    /**
     * @return True if wheel spinning is finished. False if wheel spinning is running.
     */
    public static boolean updateMotorSpeed() {
        updateRunTime();
        if (runTime >= 0 && runTime < FIRST_STAGE_TIME) {
            //First Stage
            wheelMover.setPower(getFirstStageMotorSpeed(runTime));
            return false;
        } else {
            //Ending
            wheelMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return true;
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
}