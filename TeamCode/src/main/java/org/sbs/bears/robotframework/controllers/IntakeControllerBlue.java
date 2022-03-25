package org.sbs.bears.robotframework.controllers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.sbs.bears.robotframework.enums.IntakeState;

@Config
public class IntakeControllerBlue implements IntakeController{

    private Servo scooper;
    private DcMotor compliantWheel;
    public DistanceSensor distanceSensor;
    private Servo sweeper;
    private Servo stopper;
    public Servo dumperServo;


    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
//    private double[] basePos = {.025, 0.7}; //.141

    public static double dumpPosdouble = .385;
    private double[] basePos = {.975, 1};
    private double[] parkPos = {.45, 0}; //.375 for gobilda servo
    public static double[] dumpPos = {dumpPosdouble, 0}; //??????? messed up .375 is the same???
    private double[] reversePos = {.975, -1}; //75

    /** Distance needed to switch states (mm) **/
    private double distThreshold = 50; // was 60, but new sensor.
    private double distThreshold2 = 80;

    public double timeToOpenStopper = 300; //ms 400
    public static double timeToCloseBucket = 360; //ms 650
    public double timeToPushSweeper = 230; //ms 400
    public double timeToResetSweeper = 440;

    private double sweeperOut = .088; //.725;
    private double sweeperIn = .338; //1;
    public static double stopperClosed = 0.26;  // was .3 //TODO //.3
    private double stopperOpen = 0.1; //TODO
    private boolean qIsObjectInPayload = false;

    public volatile IntakeState state = IntakeState.BASE;
    Object stateMutex = new Object();

    /** Initialization **/
    public IntakeControllerBlue(HardwareMap hardwareMap, Servo dumperServo, Telemetry telemetry) {
        /** Different hardwareMap depending on the intake side. **/
        scooper = hardwareMap.get(Servo.class, "bi");
        compliantWheel = hardwareMap.get(DcMotor.class, "leftodom");
        distanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bd");
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
        qIsObjectInPayload = (distanceSensor.getDistance(DistanceUnit.MM) < distThreshold && state == IntakeState.BASE);
        return qIsObjectInPayload;
    }

    public boolean isObjectInPayload2() {

        qIsObjectInPayload = distanceSensor.getDistance(DistanceUnit.MM) < distThreshold2;
        return qIsObjectInPayload;
    }

    /** Marc's autonomous adjusted method: do not touch or use outside of the auton brain pls.*/
    public void loadItemIntoSlideForAutonomousOnly() {
        setState(IntakeState.DUMP);

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
            case REVERSE:
                reversePos[0] = x;
                break;
            case PARK:
                parkPos[0] = x;
        }
    }

    /** Accessor for current state **/
    public IntakeState getState(){return state;}

    public double getServoPos(){return scooper.getPosition();}



    /**
     * Assigns position and motor power to their respective states \
     * **/
    private void doStateAction(){
       new Thread(() -> {
           switch (state) {
               case BASE:
                   scooper.setPosition(basePos[0]);
                   sweeper.setPosition(sweeperIn);
                   stopper.setPosition(stopperClosed);
                   compliantWheel.setPower(basePos[1]);
                   dumperServo.setPosition(SlideController.dumperPosition_READY);
                   break;

               case DUMP:
                   // maybe add here a thing to open the bucket
                   dumperServo.setPosition(SlideController.dumperPosition_READY);
                   //compliantWheel.setPower(dumpPos[1]);
                   scooper.setPosition(dumpPos[0]);
                   try {
                       Thread.sleep((long)timeToOpenStopper);
                   } catch (InterruptedException e) {
                       e.printStackTrace();
                   }
                   compliantWheel.setPower(dumpPos[1]);
                   stopper.setPosition(stopperOpen);
                   try {
                       Thread.sleep((long)timeToPushSweeper);
                   } catch (InterruptedException e) {
                       e.printStackTrace();
                   }
                   sweeper.setPosition(sweeperOut);
                   try {
                       Thread.sleep((long)timeToCloseBucket);
                   } catch (InterruptedException e) {
                       e.printStackTrace();
                   }

                   dumperServo.setPosition(SlideController.dumperPosition_CLOSED);
                    // reset position of sweeper!
                   try {
                       Thread.sleep((long)timeToResetSweeper);
                   } catch (InterruptedException e) {
                       e.printStackTrace();
                   }
                   sweeper.setPosition(sweeperIn);
                   break;

               case REVERSE:
                   scooper.setPosition(reversePos[0]);
                   compliantWheel.setPower(reversePos[1]);
                   try {
                       Thread.sleep(200);
                   } catch (InterruptedException e) {
                       e.printStackTrace();
                   }
                   sweeper.setPosition(sweeperIn);
                   break;
               case PARK:
                   scooper.setPosition(parkPos[0]);
                   compliantWheel.setPower(parkPos[1]);
                   sweeper.setPosition(sweeperIn);
                   stopper.setPosition(stopperClosed);
           }
       }).start();
    }

    public boolean isDown() {
        return (state == IntakeState.BASE);
    };

    public boolean isReversed() {
        return (state == IntakeState.REVERSE);
    };
}

