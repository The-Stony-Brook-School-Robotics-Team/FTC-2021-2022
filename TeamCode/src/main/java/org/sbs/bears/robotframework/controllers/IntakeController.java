package org.sbs.bears.robotframework.controllers;
import static org.sbs.bears.robotframework.enums.IntakeSide.FRONT;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.IntakeSide;

public class IntakeController {

    private Servo scooper;
    private DcMotor compliantWheel;
    private Rev2mDistanceSensor distanceSensor;

    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
    private double[] basePos = {.04, 0.7}; //.141
    private double[] dumpPos = {.87, 0.4};
    private double[] parkPos = {.75, 0.4};

    /** Distance needed to switch states (mm) **/
    private double distThreshold = 50.0;

    private boolean qIsObjectInPayload = false;

    volatile IntakeState state = IntakeState.BASE;
    Object stateMutex = new Object();

    /** Initialization **/
    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry, IntakeSide side) {
        /** Different hardwareMap depending on the intake side. **/
        switch(side){
            case FRONT:
                scooper = hardwareMap.get(Servo.class, "servo");
                compliantWheel = hardwareMap.get(DcMotor.class, "motor");
                distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "2m");
                scooper.setDirection(Servo.Direction.REVERSE);
                compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case BACK:
                scooper = hardwareMap.get(Servo.class, "servoBack");
                compliantWheel = hardwareMap.get(DcMotor.class, "motorBack");
                distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "2mBack");
                scooper.setDirection(Servo.Direction.FORWARD);
                compliantWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        compliantWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    /** Returns if the distance sensor reads less than distThreshold **/
    public boolean isObjectInPayload() {

        qIsObjectInPayload = distanceSensor.getDistance(DistanceUnit.MM) < distThreshold;
        return qIsObjectInPayload;
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
        setState(IntakeState.PARK);
    }

    /** TeleOp method-- checks if object is seen. If so, dumps and sets to park. **/
    public void checkIntake(){
        if(state == IntakeState.BASE && isObjectInPayload()){
            setState(IntakeState.DUMP);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            setState(IntakeState.PARK);

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

    /** Accessor for current state **/
    public IntakeState getState(){return state;}



    /** Assigns position and motor power to their respective states **/
    private void doStateAction(){
        switch(state){
            case BASE:
                scooper.setPosition(basePos[0]);
                compliantWheel.setPower(basePos[1]);
                return;

            case DUMP:
                scooper.setPosition(dumpPos[0]);
                compliantWheel.setPower(dumpPos[1]);
                return;

            case PARK:
                scooper.setPosition(parkPos[0]);
                compliantWheel.setPower(parkPos[1]);
                return;
        }
    }
}

