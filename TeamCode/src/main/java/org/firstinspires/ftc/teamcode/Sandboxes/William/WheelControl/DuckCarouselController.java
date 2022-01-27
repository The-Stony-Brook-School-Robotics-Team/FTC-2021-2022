package org.firstinspires.ftc.teamcode.Sandboxes.William.WheelControl;

import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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

public class DuckCarouselController {

    //MAGICAL_CONSTANT should be between 0.30 to 0.40. Because of the lack of enough torque, the wheel actually never achieve the ideal acceleration.
    public double MAGICAL_CONSTANT = 0.38;
    public double MAGICAL_CONSTANT_TIME_OFFSET = 0.02;

    public double FIRST_STAGE_TIME_FLAG = -1;
    public double FIRST_STAGE_TIME_INTERVAL = 1.5;
    public double SECOND_STAGE_TIME_FLAG = -1;
    public double SECOND_STAGE_TIME_INTERVAL = 0.1;
    public double currentTime;
    public double initTime;
    public double runTime;

    public DcMotor wheelMover;


    public DuckCarouselController(HardwareMap hardwareMap, Telemetry telemetry) {
        wheelMover = hardwareMap.get(DcMotor.class, "duck");
        wheelMover.setPower(0);
    }

    public void spinOneDuck() {
        initTime = getCurrentSystemSecond();
        FIRST_STAGE_TIME_FLAG = initTime + FIRST_STAGE_TIME_INTERVAL;
        SECOND_STAGE_TIME_FLAG = FIRST_STAGE_TIME_FLAG + SECOND_STAGE_TIME_INTERVAL;
        while (updateMotorSpeed()) ;
    }

    private double getCurrentSystemSecond() {
        return NanoClock.system().seconds();
    }


    /**
     * @return True if wheel spinning is finished. False if wheel spinning is running.
     */
    private boolean updateMotorSpeed() {
        currentTime = getCurrentSystemSecond();
        if (currentTime >= initTime && currentTime < FIRST_STAGE_TIME_FLAG) {
            //First Stage
            wheelMover.setPower(getFirstStageMotorSpeed());
            return true;
        } else if (currentTime >= FIRST_STAGE_TIME_FLAG && currentTime < SECOND_STAGE_TIME_FLAG) {
            wheelMover.setPower(-1);
            return true;
        } else {
            //Ending
            wheelMover.setPower(0);
            wheelMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return false;
        }
    }

    private void updateCurrentTime() {
        currentTime = getCurrentSystemSecond();
    }

    /**
     * Control the motor speed for the first stage.
     *
     * @return The speed of the real-time target speed of the motor.
     */
    private double getFirstStageMotorSpeed() {
        return MAGICAL_CONSTANT * (getCurrentSystemSecond() - initTime) + MAGICAL_CONSTANT_TIME_OFFSET;
    }
}