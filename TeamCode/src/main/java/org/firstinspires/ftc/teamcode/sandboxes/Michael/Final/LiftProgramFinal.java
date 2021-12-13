package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="S - LiftProgramFinal", group="Linear Opmode")
public class LiftProgramFinal extends LinearOpMode {

    private Servo scooper = null;
    private DcMotor compliantWheel = null;

    /** Arrays of state positions. Scooper, then motor. 1 is sky, 0 is ground. **/
    private double[] basePos = {.15, 1.0};
    private double[] dumpPos = {.55, 0.0};
    private double[] parkPos = {.5, 0.0};



    volatile LiftStates state = LiftStates.PARK;

    Object stateMutex = new Object();


    @Override

    public void runOpMode() throws InterruptedException {

        scooper = hardwareMap.get(Servo.class, "servo");
        compliantWheel = hardwareMap.get(DcMotor.class, "motor");

        scooper.setDirection(Servo.Direction.FORWARD);
        compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        compliantWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        /** Runs independent of state. **/
        new Thread(()->{
            while(opModeIsActive()) {
                telemetry.addData("Scooper position: ", scooper.getPosition());
                telemetry.addData("Wheel power: ", compliantWheel.getPower());
                telemetry.update();

            }
        }).start();


        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a && !(state == LiftStates.DUMP)) {
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
    public void doStates(){
        switch(state){
            case BASE:
                scooper.setPosition(basePos[0]);
                compliantWheel.setPower(basePos[1]);
                return;

            case DUMP:
                scooper.setPosition(dumpPos[0]);
                compliantWheel.setPower(dumpPos[1]);

            case PARK:
                scooper.setPosition(parkPos[0]);
                compliantWheel.setPower(parkPos[1]);
        }
    }
}

enum LiftStates {
    PARK,
    DUMP,
    BASE
}