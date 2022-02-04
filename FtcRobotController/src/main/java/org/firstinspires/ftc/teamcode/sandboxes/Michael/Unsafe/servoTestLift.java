package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="2021 Nissan GT-R®", group="Linear Opmode")

public class servoTestLift extends LinearOpMode {

    private Servo servo = null;
    private boolean pressingUp = false;
    private boolean pressingDown = false;
    double servoPos = 0;



    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "servo");

        servo.setDirection(Servo.Direction.FORWARD);
        servo.setPosition(servoPos);




        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.dpad_up && !pressingUp){pressingUp = true;}
            if(!gamepad1.dpad_up && pressingUp){
                servoPos+=.05;
                servo.setPosition(servoPos);
                pressingUp = false;
            }
            if(gamepad1.dpad_down && !pressingDown){pressingDown = true;}
            if(!gamepad1.dpad_down && pressingDown){
                servoPos-=.05;

                servo.setPosition(servoPos);
                pressingDown = false;
            }
            telemetry.addData("position: ", servoPos);
            telemetry.update();
        }
    }
}

