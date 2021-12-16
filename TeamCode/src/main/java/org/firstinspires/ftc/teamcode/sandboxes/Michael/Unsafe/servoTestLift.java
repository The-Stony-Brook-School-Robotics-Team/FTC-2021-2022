package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="S - servoTestLift", group="Linear Opmode")

public class servoTestLift extends LinearOpMode {
    boolean pressingA = false;
    boolean pressingB = false;
    boolean pressingUp = false;
    boolean pressingDown = false;
    private Servo servo = null;
    private DcMotor motor = null;
    private double servoPos;


    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "motor");

        servo.setDirection(Servo.Direction.FORWARD); //scooper

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        servoPos = servo.getPosition();
        servo.setPosition(.14);
        motor.setPower(.7);
        //servo.setPosition(.95);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("pos1", servo.getPosition());
        telemetry.update();
        waitForStart();
        /**lowest scooper 0, highest .6
         lowest compliant .85, highest .5
         .55 up lol .15 down **/

        while (opModeIsActive()) {
        /**    if (servoPos > 1) {
                servoPos = 1;
            }
            if (servoPos < 0) {
                servoPos = 0;
            }
            if (gamepad1.dpad_up && !pressingUp) {
                pressingUp = true;
            } else if (!gamepad1.dpad_up && pressingUp) {
                servoPos -= .05;
                servo.setPosition(servoPos);

                pressingUp = false;
            }
            if (gamepad1.dpad_down && !pressingDown) {
                pressingDown = true;
            } else if (!gamepad1.dpad_down && pressingDown) {
                servoPos += .05;
                servo.setPosition(servoPos);

                pressingDown = false;
            } */


                 if(gamepad1.a && !pressingA){
             pressingA = true;
             }
             else if(!gamepad1.a && pressingA){
             servo.setPosition(.57); //was .55
             motor.setPower(.4);
             //Thread.sleep(1000);
             //servo2.setPosition(.5);`


             pressingA = false;

             }
             if(gamepad1.b && !pressingB){
             pressingB = true;
             }
             else if(!gamepad1.b && pressingB){
                 motor.setPower(1);
                 servo.setPosition(.2125);
                 for(double i = .2125; i > .141; i-=.0001) {
                    telemetry.addData("Servo Position 2: ", servo.getPosition());
                    telemetry.update();
                    servo.setPosition(i);
                 }

                 pressingB = false;

             }

            telemetry.addData("Servo Position: ", servo.getPosition());
            telemetry.update();


        }
    }
}
