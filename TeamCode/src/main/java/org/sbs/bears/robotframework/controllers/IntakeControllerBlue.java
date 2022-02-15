package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeState;


public class IntakeControllerBlue {

    private Servo scooper;
    private DcMotor compliantWheel;
    public Rev2mDistanceSensor distanceSensor;
    private Servo sweeper;
    private Servo stopper;
    public Servo dumperServo;


    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
//    private double[] basePos = {.025, 0.7}; //.141
    private double[] basePos = {.03, 1};

    private double[] dumpPos = {.375, 0}; //.45 //.41
    private double[] parkPos = {.33, 0.0}; //75

    /** Distance needed to switch states (mm) **/
    private double distThreshold = 60;

    public long sleepAmount = 400;
    private double sweeperOut = .75;
    private double sweeperIn = 1;
    private double stopperClosed = 0.3; //TODO
    private double stopperOpen = 0.1; //TODO
    private boolean qIsObjectInPayload = false;

    volatile IntakeState state = IntakeState.BASE;
    Object stateMutex = new Object();

    /** Initialization **/
    public IntakeControllerBlue(HardwareMap hardwareMap, Servo dumperServo, Telemetry telemetry) {
        /** Different hardwareMap depending on the intake side. **/
        scooper = hardwareMap.get(Servo.class, "bi");
        compliantWheel = hardwareMap.get(DcMotor.class, "leftodom");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "bd");
        sweeper = hardwareMap.get(Servo.class, "sweep");
        stopper = hardwareMap.get(Servo.class, "bs");
        this.dumperServo = dumperServo;
        scooper.setDirection(Servo.Direction.FORWARD);
        compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        sweeper.setDirection(Servo.Direction.FORWARD);
        compliantWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    /** Returns if the distance sensor reads less than distThreshold **/
    public boolean isObjectInPayload() {

        qIsObjectInPayload = distanceSensor.getDistance(DistanceUnit.MM) < distThreshold;
        return qIsObjectInPayload;
    }

    /** Marc's autonomous adjusted method: do not touch or use outside of the auton brain pls.*/
    public void loadItemIntoSlideForAutonomousOnly() {
        setState(IntakeState.DUMP);
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //setState(IntakeState.PARK);

    }

    /** Autonomous method-- waits until object is seen, dumps, then sets to park. **/
    @Deprecated
    public void waitForIntake() {
        if(state != IntakeState.BASE){setState(IntakeState.BASE);}
        while(!isObjectInPayload()){
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        setState(IntakeState.DUMP);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setState(IntakeState.PARK);
    }

    /** TeleOp method-- checks if object is seen. If so, dumps and sets to park. **/
    public void checkIntake(){
        if(state == IntakeState.BASE && isObjectInPayload()){
            setState(IntakeState.DUMP);


        }
    }

    /**
     State setter.
     @param intakeState The desired intake state to set to the robot.
     **/

    public void setState(IntakeState intakeState) {
        synchronized (stateMutex) {
            state = intakeState;
        }
        doStateAction();
    }

    /**
     * Changes the position of the servo at a given state.
     * @param x The new state positiion, expressed 0-1 as the servo's range.
     * @param intakeState The state that's position is getting altered.
     */
    public void changeStatePosiiton(IntakeState intakeState, double x){
        switch(intakeState){
            case BASE:
                basePos[0] = x;
                break;
            case DUMP:
                dumpPos[0] = x;
                break;
            case PARK:
                parkPos[0] = x;
                break;
        }
    }

    /** Accessor for current state **/
    public IntakeState getState(){return state;}

    public double getServoPos(){return scooper.getPosition();}



    /**
     * Assigns position and motor power to their respective states \
     * **/
    private void doStateAction(){
        switch(state){
            case BASE:
                scooper.setPosition(basePos[0]);
                sweeper.setPosition(sweeperIn);
                stopper.setPosition(stopperClosed);
                compliantWheel.setPower(basePos[1]);
                dumperServo.setPosition(SlideController.dumperPosition_READY);
                break;

            case DUMP:
                compliantWheel.setPower(dumpPos[1]);
                scooper.setPosition(dumpPos[0]);
                try {
                    Thread.sleep(sleepAmount);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                stopper.setPosition(stopperOpen);
                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                sweeper.setPosition(sweeperOut);
                try {
                    Thread.sleep(750);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                dumperServo.setPosition(SlideController.dumperPosition_CLOSED);

                break;

            case PARK:
                scooper.setPosition(parkPos[0]);
                compliantWheel.setPower(parkPos[1]);

                break;
        }
    }

    public boolean isDown() {
        if(this.state == IntakeState.BASE) {
            return true;
        } else if(this.state == IntakeState.DUMP) {
            return false;
        }
    };

}

