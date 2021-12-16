package org.sbs.bears.robotframework.controllers;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.LiftStates;

public class IntakeController {

    private Servo scooper;
    private DcMotor compliantWheel;
    private Rev2mDistanceSensor rev;
    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
    private double[] basePos = {.141, 0.7};
    private double[] dumpPos = {.56, 0.4};
    private double[] parkPos = {.53, 0.4};

    /** Distance needed to switch states (mm) **/
    private double distThreshold = 50.0;

    private boolean qIsObjectInPayload = false;

    volatile LiftStates state = LiftStates.BASE;
    Object stateMutex = new Object();

    /** Initialization **/
    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        scooper = hardwareMap.get(Servo.class, "servo");
        compliantWheel = hardwareMap.get(DcMotor.class, "motor");
        rev = hardwareMap.get(Rev2mDistanceSensor.class, "2m");

        scooper.setDirection(Servo.Direction.FORWARD);
        compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        compliantWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Returns if the distance sensor reads less than distThreshold **/
    public boolean isObjectInPayload() {
        qIsObjectInPayload = rev.getDistance(DistanceUnit.MM) < distThreshold;
        return qIsObjectInPayload;
    }


    /** Autonomous method-- waits until object is seen, dumps, then sets to park. **/
    public void waitForIntake() {
        if(state != LiftStates.BASE){setState(LiftStates.BASE);}
        while(!isObjectInPayload()){
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        setState(LiftStates.DUMP);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setState(LiftStates.PARK);
    }

    /** TeleOp method-- checks if object is seen. If so, dumps and sets to park. **/
    public void checkIntake(){
        if(state == LiftStates.BASE && isObjectInPayload()){
            setState(LiftStates.DUMP);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            setState(LiftStates.PARK);

        }
    }

    /**
     State setter.
     @param liftState The desired intake state to set to the robot.
     **/
    public void setState(LiftStates liftState) {
        synchronized (stateMutex) {
            state = liftState;
        }
        doStateAction();
    }

    /** Accessor for current state **/
    public LiftStates getState(){return state;}


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

