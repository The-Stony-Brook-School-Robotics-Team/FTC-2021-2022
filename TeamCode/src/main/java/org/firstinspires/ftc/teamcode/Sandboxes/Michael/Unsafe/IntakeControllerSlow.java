package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeSide;
import org.sbs.bears.robotframework.enums.IntakeState;

public class IntakeControllerSlow {

    private Servo scooper;
    private DcMotor compliantWheel;
    private Rev2mDistanceSensor distanceSensor;

    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
//    private double[] basePos = {.025, 0.7}; //.141 // COMMENTED OUT BY MARC ON SUN JAN 9 2022 AT 22h12m54s

    private double[] basePos = {.03, 0.7}; //.141 // CHANGED BY MARC ON SUN JAN 9 2022 AT 22h12m54s

    private double[] dumpPos = {.45, 0.4}; //.87
    private double[] parkPos = {.39, 0.0}; //75

    /** Distance needed to switch states (mm) **/
    private double distThreshold = 50.0;

    private boolean qIsObjectInPayload = false;

    volatile IntakeState state = IntakeState.BASE;
    Object stateMutex = new Object();

    /** Initialization **/
    public IntakeControllerSlow(HardwareMap hardwareMap, Telemetry telemetry, IntakeSide side) {
        /** Different hardwareMap depending on the intake side. **/
        switch(side){
            case RED:
                scooper = hardwareMap.get(Servo.class, "ri");
                compliantWheel = hardwareMap.get(DcMotor.class, "rightodom");
                distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rd");
                scooper.setDirection(Servo.Direction.FORWARD);
                compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case BLUE:
                scooper = hardwareMap.get(Servo.class, "bi");
                compliantWheel = hardwareMap.get(DcMotor.class, "leftodom");
                distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "bd");
                scooper.setDirection(Servo.Direction.FORWARD);
                compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        }
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
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        setState(IntakeState.PARK);
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
                Thread.sleep(1250);
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
                if(scooper.getPosition() == basePos[0]){
                    return;
                }
                compliantWheel.setPower(basePos[1]);
                for(double i = scooper.getPosition(); i > basePos[0]; i-=.0001){
                    scooper.setPosition(Range.clip(i, basePos[0], 1));
                }
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

