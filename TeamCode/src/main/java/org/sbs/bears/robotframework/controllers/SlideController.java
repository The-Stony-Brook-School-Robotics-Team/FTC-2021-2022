package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.enums.SlideState;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class SlideController {
    private Servo verticalServo;
    private Servo horizontalServo;
    private Servo dumperServo;

    private DcMotor slideMotor;


    // TODO: determine which states are necessary and create variables and state machine

    SlideState slideState = SlideState.PARKED; // low-level steps
    SlideTarget targetParams = SlideTarget.NA;
    private boolean flagToLeave = false;


    public SlideController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        verticalServo = hardwareMap.get(Servo.class, "vt");
        horizontalServo = hardwareMap.get(Servo.class, "hz");
        dumperServo = hardwareMap.get(Servo.class, "du");
        slideMotor = hardwareMap.get(DcMotor.class, "spool");


        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0); // should be where it reset to
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void extendDropRetract(SlideTarget target)
    {
        this.targetParams = target;
        extendSlide();
        if(flagToLeave) {
            return;
        }
        dropCube();
        retractSlide();
        this.targetParams = SlideTarget.NA;
    }

    public void dropCube()
    {
        if(slideState == SlideState.OUT_FULLY) {
            dumperServo.setPosition(dumperPosition_DUMP);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            dumperServo.setPosition(dumperPosition_HOLDBLOCK);
        }
        else {
            Log.d("SlideController","Assert for slide out fully failed; dont release the cube in transit!");
        }
    }

    public void retractSlide() {
        slideState = SlideState.RET_BUCKET_OUT;
        doStateAction();
        slideState = SlideState.RET_BUCKET_IN;
        doStateAction();
        slideState = SlideState.PARKED;
    }

    public void extendSlide() {
        slideState = SlideState.EXT_BUCKET_IN;
        doStateAction();
        slideState = SlideState.EXT_BUCKET_OUT;
        doStateAction();
        if(flagToLeave)
        {
            slideState = SlideState.RET_BUCKET_IN;
            doStateAction();
            slideState = SlideState.PARKED;
            return;
        }
        slideState = SlideState.OUT_FULLY;
    }

    // this method will execute tasks associated with a transition state such as extending and retracting.
    // position states such as parked and out fully have no associated operations.
    private void doStateAction()
    {
        int targetPos = 0;
        int targetPosFinal = 0;
        double verticalServoTargetPos = 0;
        switch(slideState) {
            case PARKED:
            case OUT_FULLY:
                slideMotor.setPower(slideMotorPowerStill);
                return;
            case EXT_BUCKET_IN:
                slideMotor.setPower(slideMotorPowerMoving);
                targetPos = slideMotorPosition_BUCKET_OUT;
                slideMotor.setTargetPosition(targetPos);
                while (slideMotor.getCurrentPosition() < targetPos) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                slideMotor.setPower(slideMotorPowerStill);
                return;
            case EXT_BUCKET_OUT:
                slideMotor.setPower(slideMotorPowerStill);
                switch (targetParams) {
                    case ONE_CAROUSEL:
                        targetPosFinal = slideMotorPosition_ONE_CAROUSEL;
                        verticalServoTargetPos = vertServoPosition_ONE_CAROUSEL;
                        break;
                    case TWO_CAROUSEL:
                        targetPosFinal = slideMotorPosition_TWO_CAROUSEL;
                        verticalServoTargetPos = vertServoPosition_TWO_CAROUSEL;
                        break;
                    case THREE_CAROUSEL:
                        targetPosFinal = slideMotorPosition_THREE_CAROUSEL;
                        verticalServoTargetPos = vertServoPosition_THREE_CAROUSEL;
                        break;
                    case THREE_DEPOSIT:
                        targetPosFinal = slideMotorPosition_THREE_DEPOSIT;
                        verticalServoTargetPos = vertServoPosition_THREE_DEPOSIT;
                        break;
                    case NA:
                        Log.d("SlideController","Slide Extension failed: did not specify target. Exiting");
                        flagToLeave = true;
                        return;
                }
                slideMotor.setPower(slideMotorPowerMoving);
                slideMotor.setTargetPosition(targetPosFinal);
                setHeightToParams(verticalServoTargetPos);
                while (slideMotor.isBusy()) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                slideMotor.setPower(slideMotorPowerStill);
                return;
            case RET_BUCKET_OUT:
                targetPos = slideMotorPosition_BUCKET_OUT;
                slideMotor.setPower(slideMotorPowerMoving);
                slideMotor.setTargetPosition(targetPos);
                setHeightToParams(vertServoPosition_PARKED);
                while (slideMotor.isBusy()) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                slideMotor.setPower(slideMotorPowerStill);
                return;
            case RET_BUCKET_IN:
                targetPos = slideMotorPosition_PARKED;
                slideMotor.setPower(slideMotorPowerMoving);
                slideMotor.setTargetPosition(targetPos);
                while (slideMotor.isBusy()) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                slideMotor.setPower(slideMotorPowerStill);
                return;
        }
    }


    private void setHeightToParams(double targetPos) {
        if(slideState == SlideState.OUT_FULLY || slideState == SlideState.RET_BUCKET_OUT || slideState == SlideState.EXT_BUCKET_OUT) {
            double currentPos = verticalServo.getPosition();
            boolean qNeedsToGoUp = (currentPos < targetPos);
            if(qNeedsToGoUp) {
                for(double i = verticalServo.getPosition(); i < targetPos; i+=incrementDelta){
                    verticalServo.setPosition(Range.clip(i, targetPos, 1));
                }
            }
            else {
                for(double i = verticalServo.getPosition(); i > targetPos; i-=incrementDelta){
                    verticalServo.setPosition(Range.clip(i, targetPos, 0));
                }
            }
        }
        else {
            Log.d("SlideController","Assert for slide box outside of robot failed; dont lift the slide while the box is inside the robot!");
        }
    }



    // TODO MEASURE ALL CONSTANTS

    // vertical servo
    double vertServoPosition_PARKED = 0;
    double vertServoPosition_ONE_CAROUSEL = 0;
    double vertServoPosition_TWO_CAROUSEL = 0;
    double vertServoPosition_THREE_CAROUSEL = 0;
    double vertServoPosition_THREE_DEPOSIT = 0;
    double incrementDelta = 0.001;

    // dumper servo
    double dumperPosition_DUMP = 0;
    double dumperPosition_HOLDBLOCK = 0;

    // slide motor
    int slideMotorPosition_PARKED = 0;
    int slideMotorPosition_BUCKET_OUT = 0; // minimum position for the bucket to be out
    int slideMotorPosition_THREE_DEPOSIT = 0;
    int slideMotorPosition_THREE_CAROUSEL = 0;
    int slideMotorPosition_TWO_CAROUSEL = 0;
    int slideMotorPosition_ONE_CAROUSEL = 0;

    double slideMotorPowerMoving = 0;
    double slideMotorPowerStill = 0;










}
