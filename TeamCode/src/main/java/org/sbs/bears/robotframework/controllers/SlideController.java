package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.Beta;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.enums.SlideState;
import org.sbs.bears.robotframework.enums.SlideTarget;

public class SlideController {
    public Servo verticalServo;
    private Servo horizontalServo;
    private Servo dumperServo;
    DigitalChannel magswitch;


    public DcMotorEx slideMotor;

    public SlideState slideState = SlideState.PARKED;
    public SlideTarget targetParams = SlideTarget.NA;
    private boolean flagToLeave = false;



    public SlideController(HardwareMap hardwareMap, Telemetry telemetry)
    {
        /** Initialization **/
        magswitch = hardwareMap.get(DigitalChannel.class, "stop");
        magswitch.setMode(DigitalChannel.Mode.INPUT);
        verticalServo = hardwareMap.get(Servo.class, "vt");
        // horizontalServo = hardwareMap.get(Servo.class, "hz");
        dumperServo = hardwareMap.get(Servo.class, "du");
        slideMotor = hardwareMap.get(DcMotorEx.class, "spool");


        slideMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,new PIDFCoefficients(10,0,0,0));
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(0); // should be where it reset to
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /** Extracts the slide, drops, then retracts.
     *
     * @param target the target which represents the height and distance the slide should extend to.
     */
    public void extendDropRetract(SlideTarget target)
    {
        this.targetParams = target;
        extendSlide();
        if(flagToLeave) {
            return;
        }

        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(flagToLeave) {
            return;
        }

        dropCube();
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        retractSlide();
        this.targetParams = SlideTarget.NA;
    }


    /** An overload of extendDropRetract that allows for interruption via the gamepad.
     *
     * @param target the target which represents height and distance the slide should extend to.
     * @param gamepad the gamepad object needed to handle button presses.
     */
    public void extendDropRetract(SlideTarget target, Gamepad gamepad)
    {
        this.targetParams = target;
        extendSlide();
        if(flagToLeave) {
            return;
        }
        //Sleeps to avoid race conditions. Do not remove, might be possible to lessen
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if(flagToLeave) {
            return;
        }

        //Only proceed to drop and retract if not interrupted by a button
        //TODO fix
        if(!gamepad.x){
            dropCube();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            retractSlide();
        }
    }

    /** Moves the bucket servo to drop the cube, then returns the servo to original position. **/
    public void dropCube()
    {

        //Checks to make sure the bucket is outside of the slide before dumping
        if(slideMotor.getCurrentPosition() > slideMotorPosition_BUCKET_OUT) {
            dumperServo.setPosition(dumperPosition_DUMP);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            dumperServo.setPosition(dumperPosition_HOLDBLOCK);
        }
    }

    /** Sets the slide state to retract, does the action for this state (retracts), then sets the state to park. **/
    public void retractSlide() {
        slideState = SlideState.RETRACTING;
        doStateAction();
        Log.d("SlideController","Should be inside " + slideMotor.getCurrentPosition() + " " + verticalServo.getPosition());
        slideState = SlideState.PARKED;
    }

    /** Sets the slide state to extend, does the action for this state (extends), then sets the state to out fully. **/
    public void extendSlide() {
        Log.d("SlideController","bucket should be out");
        slideState = SlideState.EXTENDING;
        doStateAction();
        Log.d("SlideController","slide should be extended");
        slideState = SlideState.OUT_FULLY;
    }

    /** Called after a state change in order to change the robot's physical state.
     * Resting states such as PARKED and OUT_FULLY have no associated operations.
     */
    private void doStateAction()
    {
        int targetPos = 0;
        int targetPosFinal = 0;
        double verticalServoTargetPos = 0;
        switch(slideState) {
            //Resting states, just return.
            case PARKED:
            case OUT_FULLY:
                return;
            case EXTENDING:
                //Switch to set the height and distance of extension depending on the target
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
                //Wait until the slide bucket is extended outside of the robot
                while(slideMotor.getCurrentPosition() < slideMotorPosition_BUCKET_OUT){
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                //Now that the bucket is out, start lifting the slide
                setHeightToParams(verticalServoTargetPos);
                //Wait until the slide has reached its final position
                while(slideMotor.getCurrentPosition() < targetPosFinal){
                   try {
                       Thread.sleep(10);
                   } catch (InterruptedException e) {
                       e.printStackTrace();
                   }

                }
                //Kill the motor's PID and stop so it doesn't try to correct and jitter
                hardStopReset();
                return;
            case RETRACTING:
                targetPos = slideMotorPosition_PARKED;
                slideMotor.setPower(slideMotorPowerMoving);
                slideMotor.setTargetPosition(targetPos);
                //Wait until the slide is retracted to right outside the robot
                while(slideMotor.getCurrentPosition() > slideMotorPosition_BUCKET_OUT+300){
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                //Once right outside, slow down the slide and lower it.
                slideMotor.setPower(slideMotorPowerMovingBack);
                setHeightToParams(vertServoPosition_PARKED);
                //Wait until the magnetic switch is triggered.
                while(slideMotor.isBusy()){
                    try {
                        Thread.sleep(10);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                    //Once triggered, kill the motor's PID and stop to prevent overshooting and hitting the robot.
                    if(!magswitch.getState())
                    {
                        hardStopReset();
                        break;
                    }
                }
                return;
           }
    }


    /** Increments the slide's height to the desired position. **/
    private void setHeightToParams(double targetPos) {
        //Only alter the slide's height if not parked
        //TODO if(slideState != PARKED)?
        if(slideState == SlideState.OUT_FULLY || slideState == SlideState.RETRACTING || slideState == SlideState.EXTENDING || slideState == SlideState.TELEOP) {
            double currentPos = verticalServo.getPosition();
            //If the target position is higher than our current, it means we are going up, and vice versa.
            boolean qNeedsToGoUp = (currentPos < targetPos);
            if(qNeedsToGoUp) {
                //Set the servo to a slightly higher position until it reaches its target
                for(double i = verticalServo.getPosition(); i < targetPos; i+=incrementDeltaExtend){
                    verticalServo.setPosition(Range.clip(i, 0, targetPos));

                }
            }
            else { //We are going down
                for(double i = verticalServo.getPosition(); i > targetPos; i-=incrementDeltaRetract){
                    verticalServo.setPosition(Range.clip(i, targetPos, 1));
                    //Sleep to slow things down a bit
                    //TODO can be removed with a decrease of incrementDeltaRetract?
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

    /** Sets state, position and target relative to teleOp (called in teleOp). **/
    public void initTeleop(){
        slideState = SlideState.TELEOP;
        verticalServo.setPosition(vertServoPosition_PARKED);
        this.targetParams = SlideTarget.THREE_DEPOSIT;
    }
    /** Accessor methods for the slide servo and motor. **/
    public double getVerticalServoPosition(){return verticalServo.getPosition();}
    public double getSlideMotorPosition(){return slideMotor.getCurrentPosition();}

    @Deprecated
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


    /** Sets the slide to a new position given a number of encoder ticks to alter it by.
     *
     * @param encoderTicks the number of ticks to increase or decrease the slide position.
     */
    public void incrementEncoderPosition(int encoderTicks){

        slideState = SlideState.TELEOP;

        encoderTicks += slideMotor.getCurrentPosition();
        //Checks if the position given is a position that would put the box inside of the robot
        if(encoderTicks > slideMotorPosition_FULL || encoderTicks < slideMotorPosition_PARKED){return;}

        //If the slide is to be incremented inside of the bucket and the slide height is not correct, first set it to the resting position.
        if(encoderTicks < slideMotorPosition_BUCKET_OUT && (verticalServo.getPosition() < vertServoPosition_PARKED-.01 || verticalServo.getPosition() > vertServoPosition_PARKED+.01)){
            setHeightToParams(vertServoPosition_PARKED);
        }
        slideMotor.setPower(slideMotorPowerMoving);
        slideMotor.setTargetPosition(encoderTicks);
        slideMotor.setPower(slideMotorPowerMoving);

        return;
    }

    /** Sets the slide to a new height given a fraction of the servo's range to alter it by.
     *
     * @param servoPosition A double that represents the servo's range expressed from 0 to 1.
     */
    public void incrementVerticalServo(double servoPosition){
        servoPosition += verticalServo.getPosition();
        if(servoPosition > vertServoPosition_FULL_MAX){return;}
        setHeightToParams(servoPosition);
        Log.d("Servo Increment: ", "servo " + verticalServo.getPosition());
        return;

    }


    @Beta
    public void calculateAngleAndExtensionFromPosition(Pose2d currentPos)
    {
        double deltaX = currentPos.getX() - positionOfBlueHub.getX();
        double deltaY = currentPos.getY() - positionOfBlueHub.getY();
    }

    @Beta
    private double encoderInchesToTicks(double ticks) {
        return ticks * 145.1 / .785 / 2 / Math.PI;
    }

    /** Stops any attempted PID correcting by setting the motor's desired position to itself, and resetting the runmode. **/
    private void hardStopReset(){
        //STOP!!!!!!!!!!!!
        slideMotor.setPower(0);
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition());
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor.setPower(0);
        slideMotor.setTargetPosition(slideMotor.getCurrentPosition());
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0);

    }


    // TODO MEASURE ALL CONSTANTS
    

    double vertServoPosition_PARKED = 0;
    double vertServoPosition_ONE_CAROUSEL = 0.175;
    double vertServoPosition_TWO_CAROUSEL = 0.3767; ///measured
    double vertServoPosition_THREE_CAROUSEL = 0.647;
    double vertServoPosition_THREE_DEPOSIT = 0.85; // TODO //.754; //measured
    double vertServoPosition_TWO_DEPOSIT = .65; //.92; //measured
    double vertServoPosition_ONE_DEPOSIT = .47; //.92; //measured
    double vertServoPosition_PARKED_MIN = 0;
    double vertServoPosition_PARKED_MAX = 0.3;
    double vertServoPosition_FULL_MAX = 1;

    double incrementDeltaExtend = .01;
    double incrementDeltaRetract = 0.004;

    // dumper servo
    double dumperPosition_DUMP = .91;
    double dumperPosition_HOLDBLOCK = 0;

    // slide motor
    int slideMotorPosition_PARKED =  10;
    public int slideMotorPosition_BUCKET_OUT = 310; // minimum position for the bucket to be out, measured
    int slideMotorPosition_THREE_DEPOSIT = 1360; //measured
    int slideMotorPosition_THREE_CAROUSEL = 1713;
    int slideMotorPosition_TWO_CAROUSEL = 1650;
    int slideMotorPosition_ONE_CAROUSEL = 1665;
    int slideMotorPosition_FULL = 1980;
    int slideMotorPosition_START_LOWER = 400;

    public double slideMotorPowerMoving = .5;
    public double slideMotorPowerMovingBack = 0.5;
    double slideMotorPowerStill = 0;

    double deltaZForLevel3 = 12; // in
    double deltaZForLevel2 = 5; // in
    double deltaZForLevel1 = 0; // in

    static final Vector2d positionOfBlueHub = new Vector2d(24,12);







}
