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

    private Servo scooper = null;
    private DcMotor compliantWheel = null;
    private Rev2mDistanceSensor rev = null;

    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
    private double[] basePos = {.141, 1.0};
    private double[] dumpPos = {.57, 0.4};
    private double[] parkPos = {.5, 0.1};

    /** Distance needed to switch states (mm) **/
    private double distThreshold = 50.0;

    private boolean qIsObjectInPayload = false;

    volatile LiftStates state = LiftStates.BASE;
    Object stateMutex = new Object();

    public IntakeController(HardwareMap hardwareMap, Telemetry telemetry) {
        scooper = hardwareMap.get(Servo.class, "servo");
        compliantWheel = hardwareMap.get(DcMotor.class, "motor");
        rev = hardwareMap.get(Rev2mDistanceSensor.class, "2m");

        scooper.setDirection(Servo.Direction.FORWARD);
        compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        compliantWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean isObjectInPayload()
    {
        qIsObjectInPayload = rev.getDistance(DistanceUnit.MM) < distThreshold;
        return qIsObjectInPayload;
    }

    public void intakeOne() throws InterruptedException {
        if(state != LiftStates.BASE){setState(LiftStates.BASE);}
        while(!isObjectInPayload()){Thread.sleep(50);}
        setState(LiftStates.DUMP);
        Thread.sleep(1000);
        setState(LiftStates.BASE);
    }

    public void setState(LiftStates liftState) {
        synchronized (stateMutex) {
            state = liftState;
        }
        doStateAction();
    }





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

