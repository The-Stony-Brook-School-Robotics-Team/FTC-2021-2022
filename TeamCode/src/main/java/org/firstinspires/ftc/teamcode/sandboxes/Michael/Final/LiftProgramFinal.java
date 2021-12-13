package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="S - LiftProgramFinal", group="Linear Opmode")
public class LiftProgramFinal extends LinearOpMode {

    private Servo scooper = null;
    private DcMotor compliantWheel = null;
    private Rev2mDistanceSensor rev = null;

    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
    private double[] basePos = {.141, 1.0};
    private double[] dumpPos = {.57, 0.4};
    private double[] parkPos = {.5, 0.1};

    /** Distance needed to switch states (mm) **/
    private double distThreshold = 50.0;

    volatile LiftStates state = LiftStates.BASE;
    Object stateMutex = new Object();


    @Override

    public void runOpMode() throws InterruptedException {

        scooper = hardwareMap.get(Servo.class, "servo");
        compliantWheel = hardwareMap.get(DcMotor.class, "motor");
        rev = hardwareMap.get(Rev2mDistanceSensor.class, "2m");

        scooper.setDirection(Servo.Direction.FORWARD);
        compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        compliantWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        /** Runs independent of state, for debugging purposes. **/
        new Thread(()->{
            while(opModeIsActive()) {
                telemetry.addData("Scooper position: ", scooper.getPosition());
                telemetry.addData("Wheel power: ", compliantWheel.getPower());
                telemetry.addData("Distance (mm) ", rev.getDistance(DistanceUnit.MM));
                telemetry.addData("State ", state);
                telemetry.update();

            }
        }).start();


        while (opModeIsActive() && !isStopRequested()) {
            if (collected() && !(state == LiftStates.DUMP)) {
                synchronized (stateMutex) {
                    state = LiftStates.DUMP;
                }
            }
            else if (gamepad1.b && !(state == LiftStates.BASE)){
                synchronized (stateMutex) {
                    state = LiftStates.BASE;
                }
            }
            else if (gamepad1.x && !(state == LiftStates.PARK)) {
                synchronized (stateMutex) {
                    state = LiftStates.PARK;
                }
            }


            doStates();

        }
    }
    public boolean collected(){return rev.getDistance(DistanceUnit.MM) < distThreshold;}

    public void doStates(){
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

enum LiftStates {
    PARK,
    DUMP,
    BASE
}