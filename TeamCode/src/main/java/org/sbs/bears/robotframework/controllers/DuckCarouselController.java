package org.sbs.bears.robotframework.controllers;

import android.util.Log;

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
    private static final double MAGICAL_CONSTANT = 0.3;

    private  double FIRST_STAGE_TIME;
    private  double FIRST_STAGE_TIME_INTERVAL = 1.5;
    private  double iniTime;
    private  double runTime;

    private  DcMotor wheelMover;


    public DuckCarouselController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        wheelMover = hardwareMap.get(DcMotor.class,"duck");
        initializeEnvironment();
    }
    public void spinOneDuck(boolean qBlue)
    {/*
        wheelMover.setPower(qBlue ? -.3 : .3);
        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        wheelMover.setPower(0);
*/
        //cleanup();
        initializeEnvironment();
        Log.d("DuckCtrller","start");
        while(true)
        {
            if(updateMotorSpeed())
            {
                wheelMover.setPower(0);
                return;
            }
        }
    }

    private void cleanup()
    {
        iniTime = 0;
        runTime = 0;
        FIRST_STAGE_TIME = 0;
        FIRST_STAGE_TIME_INTERVAL = 1.5;
    }
    //------------------------------------------------------------
   /* public static void spinWheel(DcMotor wheelMover) {          //
        initializeEnvironment(wheelMover);                      //
        while (updateMotorSpeed()) ;                            //
    }*/                                                           //
    //------------------------------------------------------------

    private double getCurrentSystemSecond() {
        return NanoClock.system().seconds();
    }

    public void initializeEnvironment() {
        wheelMover.setDirection(DcMotorSimple.Direction.REVERSE);
       // initializeVariables();
        Log.d("DuckCtrller","initd");
        initializeVariables();
    }

    /**
     * @return True if wheel spinning is finished. False if wheel spinning is running.
     */
    public boolean updateMotorSpeed() {
        updateRunTime();
        Log.d("DuckCtrller","update1");
        if (runTime >= 0 && runTime < FIRST_STAGE_TIME) {
            //First Stage
            Log.d("DuckCtrller","update2");
            wheelMover.setPower(getFirstStageMotorSpeed(runTime));
            return false;
        } else {
            //Ending
            Log.d("DuckCtrller","update3");
            wheelMover.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return true;
        }
    }

    private void updateRunTime() {
        runTime = getCurrentSystemSecond() - iniTime;
    }

    private void initializeVariables() {
        FIRST_STAGE_TIME = FIRST_STAGE_TIME_INTERVAL;
        iniTime = getCurrentSystemSecond();
    }

    /**
     * Control the motor speed for the first stage.
     *
     * @return The speed of the real-time target speed of the motor.
     */
    private double getFirstStageMotorSpeed(double runTime) {
        Log.d("DuckCtrller","update4 " + runTime);
        return MAGICAL_CONSTANT * runTime;
    }
}