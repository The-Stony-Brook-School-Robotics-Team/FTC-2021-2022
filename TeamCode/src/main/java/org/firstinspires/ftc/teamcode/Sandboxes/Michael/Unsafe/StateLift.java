package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="S - StateLift", group="Linear Opmode")
public class StateLift extends LinearOpMode {

    private Servo scooper = null;
    private Servo compliant = null;
    private DcMotor compliantWheel = null;

    /** Arrays of state positions. Scooper, then compliant, then motor. 1 is sky, 0 is ground. **/
    private double[] basePos = {0, .15, 1.0};
    private double[] dumpPos = {.6, .95, 0.0}; //TODO: Retest compliant positions.
    private double[] parkPos = {.5, .8, 0.0};

    //TODO: Holder array? I dont like it

    volatile LiftStates state = LiftStates.DUMP; //TODO: Add PARK state, initial/resting.

    Object stateMutex = new Object();


    @Override

    public void runOpMode() throws InterruptedException {

        scooper = hardwareMap.get(Servo.class, "servo");
        compliant = hardwareMap.get(Servo.class, "servo2");
        compliantWheel = hardwareMap.get(DcMotor.class, "motor");

        scooper.setDirection(Servo.Direction.FORWARD);
        compliant.setDirection(Servo.Direction.FORWARD);
        compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        compliantWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        /** Runs independent of state. **/
        new Thread(()->{
            while(opModeIsActive()) {
                telemetry.addData("Scooper position: ", scooper.getPosition());
                telemetry.addData("Compliant Position: ", compliant.getPosition());
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
            if (gamepad1.b && !(state == LiftStates.BASE)){
                synchronized (stateMutex) {
                    state = LiftStates.BASE;
                }
            }
            if (gamepad1.x && !(state == LiftStates.PARK)) {
                synchronized (stateMutex) {
                    state = LiftStates.PARK;
                }
            }
            if(gamepad1.y && !(state == LiftStates.STOP)) {
                synchronized (stateMutex) {
                    state = LiftStates.STOP;
                }
            }

            doStates();

        }
    }
    public void doStates(){
        switch(state){
            case BASE:
                if(scooper.getPosition() == basePos[0]){
                    return;
                }
                compliantWheel.setPower(basePos[2]);
                //TODO: Garbage variables need to be the last state's array, not just the other array.
                for(double i = dumpPos[1], j = dumpPos[0]; j > basePos[0]; i-=.001, j-=.0005){
                    compliant.setPosition(Range.clip(i, basePos[1], 1));
                    scooper.setPosition(j);
                }
                state = LiftStates.STOP;
                return;

            case DUMP:
                if(scooper.getPosition() == dumpPos[0]){
                    return;
                }
                compliantWheel.setPower(dumpPos[2]);
                //TODO: See above.
                for(double i = basePos[0], j = basePos[1]; j < dumpPos[1]; i+=.001, j+=.0005){
                    scooper.setPosition(Range.clip(i, 0, dumpPos[0]));
                    compliant.setPosition(j);
                }
                state = LiftStates.STOP;
                return;

            case PARK:
                if(scooper.getPosition() == parkPos[0]){
                    return;
                }
                compliantWheel.setPower(parkPos[2]);
                //TODO: See above.
                for(double i = basePos[0], j = basePos[1]; j < parkPos[1]; i+=.001, j+=.0005){
                    scooper.setPosition(Range.clip(i, 0, parkPos[0]));
                    compliant.setPosition(j);
                }
                state = LiftStates.STOP;
                return;
            case STOP:
                scooper.setPosition(scooper.getPosition());
                compliant.setPosition(compliant.getPosition());


        }
    }
}

enum LiftStates {
    PARK,
    DUMP,
    BASE,
    STOP
}