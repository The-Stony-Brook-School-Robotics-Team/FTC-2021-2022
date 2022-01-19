package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.enums.SlideState;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class SlideController {
    public Servo verticalServo;
    private Servo horizontalServo;
    private Servo dumperServo;

    public DcMotor slideMotor;


    // TODO: determine which states are necessary and create variables and state machine

    public SlideState slideState = SlideState.PARKED; // low-level steps
    public SlideTarget targetParams = SlideTarget.NA;
    private boolean flagToLeave = false;



    public SlideController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        verticalServo = hardwareMap.get(Servo.class, "vt");
        // horizontalServo = hardwareMap.get(Servo.class, "hz");
        dumperServo = hardwareMap.get(Servo.class, "du");
        slideMotor = hardwareMap.get(DcMotor.class, "spool");


        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0); // should be where it reset to
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //verticalServo.setDirection(Servo.Direction.REVERSE);
    }

    public void extendDropRetract(SlideTarget target)
    {
        this.targetParams = target;
        extendSlide();
        if(flagToLeave) {
            return;
        }

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //if(flagToLeave) {
          //  return;
        //}

        dropCube();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        retractSlide();
        this.targetParams = SlideTarget.NA;
    }

    public void dropCube()
    {
        if(slideMotor.getCurrentPosition() > slideMotorPosition_BUCKET_OUT) {
            dumperServo.setPosition(dumperPosition_DUMP);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            dumperServo.setPosition(dumperPosition_HOLDBLOCK);
            //}
            // else {
            //   Log.d("SlideController","Assert for slide out fully failed; dont release the cube in transit!");
        }
    }

    public void retractSlide() {
        slideState = SlideState.RET_BUCKET_OUT;
        doStateAction();
        Log.d("SlideController","Should be right outside " + slideMotor.getCurrentPosition() + " " + verticalServo.getPosition());
        slideState = SlideState.RET_BUCKET_IN;
        doStateAction();
        Log.d("SlideController","Should be inside " + slideMotor.getCurrentPosition() + " " + verticalServo.getPosition());
        slideState = SlideState.PARKED;
    }

    public void extendSlide() {
        //slideState = SlideState.EXT_BUCKET_IN;
        //doStateAction();
        Log.d("SlideController","bucket should be out");
        slideState = SlideState.EXT_BUCKET_OUT;
        doStateAction();
        Log.d("SlideController","slide should be extended");
       // if(flagToLeave)
        //{
         //   slideState = SlideState.RET_BUCKET_IN;
          //  doStateAction();
           // slideState = SlideState.PARKED;
            //return;
        //}
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
                //slideMotor.setPower(slideMotorPowerStill);
                return;
            case EXT_BUCKET_IN:

                slideMotor.setPower(slideMotorPowerMovingWBucketInside);
                targetPos = slideMotorPosition_BUCKET_OUT;
                slideMotor.setTargetPosition(targetPos);
                while (slideMotor.isBusy()) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                //slideMotor.setPower(slideMotorPowerStill);
                return;
            case EXT_BUCKET_OUT:
                //slideMotor.setPower(slideMotorPowerStill);
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
                slideMotor.setTargetPosition(targetPosFinal);
                slideMotor.setPower(slideMotorPowerMoving);
                //setHeightToParams(verticalServoTargetPos);
                while(slideMotor.getCurrentPosition() < slideMotorPosition_BUCKET_OUT){
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                setHeightToParams(verticalServoTargetPos);
                while (slideMotor.isBusy()) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                //slideMotor.setPower(slideMotorPowerStill);
                return;
            case RET_BUCKET_OUT:
                //targetPos = slideMotorPosition_BUCKET_OUT;
                targetPos = slideMotorPosition_PARKED;
                slideMotor.setPower(slideMotorPowerMoving);
                slideMotor.setTargetPosition(targetPos);
                //setHeightToParams(vertServoPosition_PARKED);
                while(slideMotor.getCurrentPosition() > slideMotorPosition_BUCKET_OUT){
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                // targetPos = slideMotorPosition_BUCKET_OUT;
                // slideMotor.setTargetPosition(targetPos);
                setHeightToParams(vertServoPosition_PARKED);
                //   while (slideMotor.isBusy()) {
                //      try {
                //           Thread.sleep(10);
                ///       } catch (InterruptedException e) {
                //            e.printStackTrace();
                //         }
                //     }

                //slideMotor.setPower(slideMotorPowerStill);
                while(slideMotor.isBusy()){
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                return;
            case RET_BUCKET_IN:
                targetPos = slideMotorPosition_PARKED;
                slideMotor.setPower(slideMotorPowerMovingWBucketInside);
                slideMotor.setTargetPosition(targetPos);
                while (slideMotor.isBusy()) {
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                //slideMotor.setPower(slideMotorPowerStill);
                return;
        }
    }



    private void setHeightToParams(double targetPos) {
        if(slideState == SlideState.OUT_FULLY || slideState == SlideState.RET_BUCKET_OUT || slideState == SlideState.EXT_BUCKET_OUT || slideState == SlideState.TELEOP) {
            double currentPos = verticalServo.getPosition();
            boolean qNeedsToGoUp = (currentPos < targetPos);
            if(qNeedsToGoUp) {
                for(double i = verticalServo.getPosition(); i < targetPos; i+=incrementDelta){
                    verticalServo.setPosition(Range.clip(i, 0, targetPos));
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
            else {
                for(double i = verticalServo.getPosition(); i > targetPos; i-=incrementDelta){
                    verticalServo.setPosition(Range.clip(i, targetPos, 1));
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        }
        else {
            Log.d("SlideController","Assert for slide box outside of robot failed; dont lift the slide while the box is inside the robot!");
        }
    }


    /** TeleOp Methods */
    public void initTeleop(){
        slideState = SlideState.TELEOP;
        verticalServo.setPosition(vertServoPosition_PARKED);
        this.targetParams = SlideTarget.TWO_CAROUSEL;
    }

    public double getVerticalServoPosition(){return verticalServo.getPosition();}
    public double getSlideMotorPosition(){return slideMotor.getCurrentPosition();}

    public void setToEncoderPosition(int encoderTicks){
        int oldPos = slideMotor.getCurrentPosition();
        slideState = SlideState.TELEOP;

        if(slideMotor.isBusy()){
            return;
        }


        //Checks if the position given is a position that would put the box inside of the robot
        if(encoderTicks > slideMotorPosition_FULL || encoderTicks < slideMotorPosition_PARKED){return;}

        if((encoderTicks < slideMotorPosition_BUCKET_OUT || oldPos < slideMotorPosition_BUCKET_OUT) && (verticalServo.getPosition() < vertServoPosition_PARKED-.01 || verticalServo.getPosition() > vertServoPosition_PARKED+.01)){
            setHeightToParams(vertServoPosition_PARKED);
        }
        slideMotor.setPower(slideMotorPowerMoving);
        slideMotor.setTargetPosition(encoderTicks);
        while(slideMotor.isBusy())
        {
            Sleep.sleep(10);
        }
        slideMotor.setPower(slideMotorPowerStill);
        return;
    }

    //TODO: i dont know why it dont be working
    public void setToInchPosition(double inches){
        slideState = SlideState.TELEOP;
        inches = encoderInchesToTicks(inches);
        //Checks if the position given is a position that would put the box inside of the robot
        if(inches>slideMotorPosition_FULL){return;}
        if((inches < slideMotorPosition_BUCKET_OUT) && (slideState != SlideState.PARKED && slideState != SlideState.EXT_BUCKET_IN)){
            Log.d("SlideController","Gave an encoder position with the box inside of the robot");
            retractSlide();
            return;
        }
        else if(inches < slideMotorPosition_BUCKET_OUT){
            Log.d("SlideController","Gave an encoder position with the box inside of the robot");
            return;
        }
        //If the slide is not extended, extend it to the minimum position.
        if(slideState == SlideState.PARKED){
            slideState = SlideState.EXT_BUCKET_IN;
            doStateAction();
            //slideState = SlideState.EXT_BUCKET_OUT;
        }
        //Checks if the slide is in a position to move
        slideMotor.setPower(slideMotorPowerMoving);
        slideMotor.setTargetPosition((int)inches);
        while(slideMotor.isBusy())
        {
            Sleep.sleep(10);
        }
        slideMotor.setPower(slideMotorPowerStill);

        return;


    }
    public void incrementEncoderPosition(int encoderTicks){

        slideState = SlideState.TELEOP;
        // if(slideMotor.isBusy()){
        //    return;
        //}


        encoderTicks += slideMotor.getCurrentPosition();
        //Checks if the position given is a position that would put the box inside of the robot
        if(encoderTicks > slideMotorPosition_FULL || encoderTicks < slideMotorPosition_PARKED){return;}

        if(encoderTicks < slideMotorPosition_BUCKET_OUT && (verticalServo.getPosition() < vertServoPosition_PARKED-.01 || verticalServo.getPosition() > vertServoPosition_PARKED+.01)){
            setHeightToParams(vertServoPosition_PARKED);
        }
        slideMotor.setPower(slideMotorPowerMoving);
        slideMotor.setTargetPosition(encoderTicks);
        slideMotor.setPower(slideMotorPowerMoving);
        // while(slideMotor.isBusy())
        //{
        //  Sleep.sleep(10);
        //}
        //if(!slideMotor.isBusy()){
        //  slideMotor.setPower(slideMotorPowerStill);
        //}
        return;
    }
    public void incrementInchPosition(double inches){
        slideState = SlideState.TELEOP;
        inches = encoderInchesToTicks(inches);
        inches += slideMotor.getCurrentPosition();
        //Checks if the position given is a position that would put the box inside of the robot
        if(inches > slideMotorPosition_FULL){return;}
        if(inches < slideMotorPosition_BUCKET_OUT){
            Log.d("SlideController","Gave an encoder position with the box inside of the robot");
            if(inches < 0){
                retractSlide();
                return;
            }
            if(inches > 0){
                extendSlide();
                return;
            }
        }
        //If the slide is not extended, extend it to the minimum position.
        if(slideState == SlideState.PARKED){
            slideState = SlideState.EXT_BUCKET_IN;
            doStateAction();
            //slideState = SlideState.EXT_BUCKET_OUT;
        }
        //Checks if the slide is in a position to move
        slideMotor.setPower(slideMotorPowerMoving);
        slideMotor.setTargetPosition((int)inches);
        while(slideMotor.isBusy())
        {
            Sleep.sleep(10);
        }
        slideMotor.setPower(slideMotorPowerStill);
        return;

    }

    public void incrementVerticalServo(double servoPosition){
        servoPosition += verticalServo.getPosition();
        if(servoPosition > vertServoPosition_FULL_MAX){return;}
        Log.d("stupid michael", "servo " + verticalServo.getPosition());
        //if((slideState == SlideState.PARKED || slideState == SlideState.EXT_BUCKET_IN) && (servoPosition > vertServoPosition_PARKED_MAX || servoPosition < vertServoPosition_PARKED_MIN)){return;}

        setHeightToParams(servoPosition);
        return;

    }
    /** End of TeleOp Methods */

    private double encoderInchesToTicks(double ticks) {
        return ticks * 145.1 / .785 / 2 / Math.PI;
    }




    // TODO MEASURE ALL CONSTANTS

    // vertical servo

    double vertServoPosition_PARKED = 0;
    double vertServoPosition_ONE_CAROUSEL = 0.26;
    double vertServoPosition_TWO_CAROUSEL = 0.47; ///measured
    double vertServoPosition_THREE_CAROUSEL = 0.762;
    double vertServoPosition_THREE_DEPOSIT = .762; // TODO //.92; //measured
    double vertServoPosition_TWO_DEPOSIT = .65; //.92; //measured
    double vertServoPosition_ONE_DEPOSIT = .47; //.92; //measured
    double incrementDelta = 0.003;
    double vertServoPosition_PARKED_MIN = 0;
    double vertServoPosition_PARKED_MAX = 0.3;
    double vertServoPosition_FULL_MAX = 1;

    // dumper servo
    double dumperPosition_DUMP = .91;
    double dumperPosition_HOLDBLOCK = 0;

    // slide motor
    int slideMotorPosition_PARKED =  0;
    public int slideMotorPosition_BUCKET_OUT = 310; // minimum position for the bucket to be out, measured
    int slideMotorPosition_THREE_DEPOSIT = 1246; //measured
    int slideMotorPosition_THREE_CAROUSEL = 1713;
    int slideMotorPosition_TWO_CAROUSEL = 1692;
    int slideMotorPosition_ONE_CAROUSEL = 1671;
    int slideMotorPosition_FULL = 1980;
    int slideMotorPosition_START_LOWER = 400;

    public double slideMotorPowerMoving = 1;
    public double slideMotorPowerMovingWBucketInside = 1;
    double slideMotorPowerStill = 0;






}
