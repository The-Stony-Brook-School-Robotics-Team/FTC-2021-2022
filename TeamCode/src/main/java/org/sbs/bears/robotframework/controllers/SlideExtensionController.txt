package org.sbs.bears.robotframework.controllers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.sbs.bears.robotframework.enums.SlideComponents;
import org.sbs.bears.robotframework.enums.SlideOverallMode;
import org.sbs.bears.robotframework.enums.SlideState;
import org.sbs.bears.robotframework.enums.SlideTarget;

import java.util.HashMap;

public class SlideExtensionController {


    private Servo verticalServo;
    private Servo horizontalServo;
    private Servo dumperServo;

    private DcMotor slideMotor;


    /** Vertical Servo, then Horizontal Servo, then Dumper Servo, then Slide Motor, */
    private int currentEncoderPosition = 0;
    private HashMap<SlideComponents, Double> values = new HashMap<>();


    private SlideOverallMode bigmode = SlideOverallMode.PARKED;
    private SlideState slideState = SlideState.PARKED;
    private SlideTarget targetParams = SlideTarget.NA;




    //set to random duck, then keep highest for auton
    volatile SlideState state;
    Object stateMutex = new Object();

    public SlideExtensionController(HardwareMap hardwareMap, Telemetry telemetry)
    {

        verticalServo = hardwareMap.get(Servo.class, "vt");
        horizontalServo = hardwareMap.get(Servo.class, "hz");
        dumperServo = hardwareMap.get(Servo.class, "du");

        slideMotor = hardwareMap.get(DcMotor.class, "spool");

        populateValues();

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_IN).intValue());
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        currentEncoderPosition = slideMotor.getCurrentPosition();

        verticalServo.setPosition(values.get(SlideComponents.VERTICAL_SERVO_MIDDLE));


    }

    /**
     State setter.
     @param slideState The desired slide state to set to the robot.
     **/

    public void setState(SlideState slideState) {
        synchronized (stateMutex) {
            state = slideState;
        }
        doStateAction();
    }

    /** Accessor for current state **/
    public SlideState getState(){return state;}

    public double getServoPosition(){return verticalServo.getPosition();}
    public int getMotorPosition(){return slideMotor.getCurrentPosition();}
    public double getMotorPower(){return slideMotor.getPower();}

    /**
     * Adds a number of ticks to the current position.
     * @param x The number to increase the motor position by in ticks.
     */
    public void setMotorIncrement(int x){
        currentEncoderPosition = slideMotor.getCurrentPosition();
        currentEncoderPosition += x;
        slideMotor.setTargetPosition(currentEncoderPosition);

    }


    /** Assigns position and motor power to their respective states **/
    private void doStateAction(){
        switch(state){





            case BOTTOM:
                slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED).intValue());
                slideMotor.setPower(values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                try{
                    Thread.sleep(1000);
                }
                catch (Exception e){

                }

                /** for(double i = verticalServo.getPosition(); i > values.get(SlideComponents.VERTICAL_SERVO_BOTTOM); i-=.0001){
                 verticalServo.setPosition(Range.clip(i, values.get(SlideComponents.VERTICAL_SERVO_BOTTOM), 1));
                 } */
                return;

            case MIDDLE:
                slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED).intValue());
                slideMotor.setPower(values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                try{
                    Thread.sleep(1000);
                }
                catch (Exception e){

                }

                for(double i = verticalServo.getPosition(); i > values.get(SlideComponents.VERTICAL_SERVO_BOTTOM); i-=.0001){
                    verticalServo.setPosition(Range.clip(i, values.get(SlideComponents.VERTICAL_SERVO_BOTTOM), 1));
                }
                return;

            case TOP:
                verticalServo.setPosition(values.get(SlideComponents.VERTICAL_SERVO_TOP));

                slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED).intValue());
                slideMotor.setPower(values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                return;

            case IN:
                verticalServo.setPosition(values.get(SlideComponents.VERTICAL_SERVO_MIDDLE));

                slideMotor.setTargetPosition(values.get(SlideComponents.SLIDE_MOTOR_POSITION_IN).intValue());
                slideMotor.setPower(-values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));


        }
    }
    private void populateValues(){
        //TODO: Record actual values.

        values.put(SlideComponents.VERTICAL_SERVO_TOP, 0.812);
        values.put(SlideComponents.VERTICAL_SERVO_MIDDLE, 0.41);
        values.put(SlideComponents.VERTICAL_SERVO_BOTTOM, 0.1);

        values.put(SlideComponents.HORIZONTAL_SERVO_POSITION, 0.5);

        values.put(SlideComponents.DUMPER_SERVO_DUMPED, 0.0);
        values.put(SlideComponents.DUMPER_SERVO_UP, 1.0);

        values.put(SlideComponents.SLIDE_MOTOR_POWER_MOVING, 1.0);
        values.put(SlideComponents.SLIDE_MOTOR_POWER_REST, 0.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED, 500.0); // placeholder bruh

        // TODO TODO TODO MEASURE PLSPLSPLS
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_IN, 25.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_MOVING_W_BUCKET_IN, 25.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED_THREE_DEPOSIT, 25.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED_TWO_DEPOSIT, 25.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED_ONE_DEPOSIT, 25.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED_ONE_CAROUSEL, 25.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED_TWO_CAROUSEL, 25.0);
        values.put(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED_THREE_CAROUSEL, 25.0);

        values.put(SlideComponents.SLIDE_MOTOR_MIN_TICKS, 700.0);
    }



    public void doSlideControlAction()
    {
        int targetPos;
        switch (state) {
            case EXT_START:
                targetPos = values.get(SlideComponents.SLIDE_MOTOR_POSITION_IN).intValue();
                slideMotor.setTargetPosition(targetPos);
                while(slideMotor.getCurrentPosition() < targetPos) {
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                state = SlideState.EXT_BUCKET_IN;
                break;
            case EXT_BUCKET_IN:
                targetPos = values.get(SlideComponents.SLIDE_MOTOR_POSITION_MOVING_W_BUCKET_IN).intValue();
                slideMotor.setTargetPosition(targetPos);
                slideMotor.setPower(values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                state = SlideState.LIFTING_N_FULLY_EXTING;
                while(slideMotor.getCurrentPosition() < targetPos) {
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                state = SlideState.LIFTING_N_FULLY_EXTING;
                break;
            case LIFTING_N_FULLY_EXTING:
                targetPos = values.get(SlideComponents.SLIDE_MOTOR_POSITION_EXTENDED).intValue();
                slideMotor.setTargetPosition(targetPos);
                slideMotor.setPower(values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                // lift slide to position
                setHeightToParams();
                while(slideMotor.getCurrentPosition() < targetPos) {
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                state = SlideState.OUT_FULLY;
                bigmode = SlideOverallMode.EXTENDED;
                break;
            case LOWERING_N_RETRACTING:
                targetPos = values.get(SlideComponents.SLIDE_MOTOR_POSITION_MOVING_W_BUCKET_IN).intValue();
                slideMotor.setTargetPosition(targetPos);
                slideMotor.setPower(-values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                // lift slide to position
                setHeightToHome();

                while(slideMotor.getCurrentPosition() > targetPos) {
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                state = SlideState.RET_BUCKET_IN;
                break;
            case RET_BUCKET_IN:
                targetPos = values.get(SlideComponents.SLIDE_MOTOR_POSITION_IN).intValue();
                slideMotor.setTargetPosition(targetPos);
                slideMotor.setPower(-values.get(SlideComponents.SLIDE_MOTOR_POWER_MOVING));
                while(slideMotor.getCurrentPosition() > targetPos) {
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                state = SlideState.PARKED;
                break;

        }
    }
    private void setHeightToHome() {
        double currentPos = verticalServo.getPosition();
        double targetPos = 0;
        for(double i = verticalServo.getPosition(); i > targetPos; i-=.0001) {
            verticalServo.setPosition(Range.clip(i, targetPos, 0));
        }
    }

    private void setHeightToParams() {
        double currentPos = verticalServo.getPosition();
        double targetPos = 0;
        switch(targetParams) {
            case NA:
                return;
            case ONE_CAROUSEL:
                targetPos = values.get(SlideComponents.VERTICAL_SERVO_ONE_CAROUSEL);
                break;
            case TWO_CAROUSEL:
                targetPos = values.get(SlideComponents.VERTICAL_SERVO_TWO_CAROUSEL);
                break;
            case THREE_CAROUSEL:
                targetPos = values.get(SlideComponents.VERTICAL_SERVO_THREE_CAROUSEL);
                break;
            case ONE_DEPOSIT:
                targetPos = values.get(SlideComponents.VERTICAL_SERVO_ONE_DEPOSIT);
                break;
            case TWO_DEPOSIT:
                targetPos = values.get(SlideComponents.VERTICAL_SERVO_TWO_DEPOSIT);
                break;
            case THREE_DEPOSIT:
                targetPos = values.get(SlideComponents.VERTICAL_SERVO_THREE_DEPOSIT);
                break;
        }
        boolean qNeedsToGoUp = (currentPos < targetPos);
        if(qNeedsToGoUp) {
            for(double i = verticalServo.getPosition(); i < targetPos; i+=.0001){
                verticalServo.setPosition(Range.clip(i, targetPos, 1));
            }
        }
        else {
            for(double i = verticalServo.getPosition(); i > targetPos; i-=.0001){
                verticalServo.setPosition(Range.clip(i, targetPos, 0));
            }
        }
    }

    public void extendSlide()
    {
        state = SlideState.EXT_START;
        while(state != SlideState.OUT_FULLY)
        {
            doSlideControlAction();
        }
    }
    public void retractSlide()
    {
        state = SlideState.LOWERING_N_RETRACTING;
        while(state != SlideState.PARKED)
        {
            doSlideControlAction();
        }
    }
    public void dropCube()
    {
        dumperServo.setPosition(cubeDropPosition);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        dumperServo.setPosition(cubeBackPosition);
    }
    public void extendDropRetract()
    {
        SlideOverallMode bigMode = SlideOverallMode.EXTENDING;
        extendSlide();
        bigMode = SlideOverallMode.EXTENDED;
        dropCube();
        bigMode = SlideOverallMode.RETRACTING;
        retractSlide();
        bigMode = SlideOverallMode.PARKED;
    }
    public void extendDropRetract(SlideTarget target)
    {
        this.targetParams = target;
        bigmode = SlideOverallMode.EXTENDING;
        extendSlide();
        dropCube();
        bigmode = SlideOverallMode.RETRACTING;
        retractSlide();
        bigmode = SlideOverallMode.PARKED;
    }

    public static double cubeDropPosition = 0; // TODO
    public static double cubeBackPosition = 0; // TODO

}
