package org.sbs.bears.robotframework.controllers;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeSide;
import org.sbs.bears.robotframework.enums.IntakeState;

public class IntakeControllerRed {

    private Servo scooper;
    private DcMotor compliantWheel;
    public Rev2mDistanceSensor distanceSensor;

    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
//    private double[] basePos = {.025, 0.7}; //.141 // COMMENTED OUT BY MARC ON SUN JAN 9 2022 AT 22h12m54s

    private double[] basePos = {.0144, 1}; //.141 // CHANGED BY MARC ON SUN JAN 9 2022 AT 22h12m54s

    private double[] dumpPos = {.375, 0}; //.4
    private double[] parkPos = {.33, 0.0}; //75

    /** Distance needed to switch states (mm) **/
    private double distThreshold = 60;

    private boolean qIsObjectInPayload = false;

    volatile IntakeState state = IntakeState.BASE;
    Object stateMutex = new Object();

    /** Initialization **/
    public IntakeControllerRed(HardwareMap hardwareMap, Telemetry telemetry) {
        /** Different hardwareMap depending on the intake side. **/
        scooper = hardwareMap.get(Servo.class, "ri");
        compliantWheel = hardwareMap.get(DcMotor.class, "rightodom");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rd");

        scooper.setDirection(Servo.Direction.FORWARD);
        compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);
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
        setState(IntakeState.DUMP);

    }

    /** Autonomous method-- waits until object is seen, dumps, then sets to park. **/
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
        setState(IntakeState.DUMP);
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
    public void changeStatePosition(IntakeState intakeState, double x){
        switch(intakeState){
            case BASE:
                basePos[0] = x;
                break;
            case DUMP:
                dumpPos[0] = x;
                break;
            case REVERSE:
                parkPos[0] = x;
                break;
        }
    }

    /** Accessor for current state **/
    public IntakeState getState(){return state;}

    public double getServoPos(){return scooper.getPosition();}



    /** Assigns position and motor power to their respective states **/
    private void doStateAction(){
        switch(state){
            case BASE:
                scooper.setPosition(basePos[0]);
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                compliantWheel.setPower(basePos[1]);
                return;

            case DUMP:
                compliantWheel.setPower(dumpPos[1]);
                scooper.setPosition(dumpPos[0]);
                return;

            case REVERSE:
                scooper.setPosition(parkPos[0]);
                compliantWheel.setPower(parkPos[1]);
                return;
        }
    }
}

