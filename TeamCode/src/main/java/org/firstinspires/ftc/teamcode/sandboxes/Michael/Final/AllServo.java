package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="B- ALL SERVOS", group="Linear Opmode")
public class AllServo extends LinearOpMode {
    private Servo redIntake;
    private Servo blueIntake;
    private Servo blueSweeper;
    private Servo verticalServo;
    private Servo dumper;

    private boolean qA = false;
    private boolean qUp = false;
    private boolean qDown = false;
    int count = 0;

    private Servo place;





    public void runOpMode() throws InterruptedException {
        redIntake = hardwareMap.get(Servo.class, "ri"); //a
        blueIntake = hardwareMap.get(Servo.class, "bi"); //b
        blueSweeper = hardwareMap.get(Servo.class, "sweep"); //x
        verticalServo = hardwareMap.get(Servo.class, "vt"); //y
        dumper = hardwareMap.get(Servo.class, "du"); //rb


        Servo[] servos = {redIntake, blueIntake, blueSweeper, verticalServo, dumper};
        String[] servoNames = {"red intake :3", "blue intake uwu", "blue sweeper x3", "vertical servo owo", "d-dumper... OwO"};
        place = servos[0];
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a && !qA){
                qA = true;
                count++;
                if(count > servos.length - 1) count = 0;
                place = servos[count];
                place.setPosition(.2);
            }
            if(!gamepad1.a && qA){
                qA = false;
            }


            if(gamepad1.dpad_up && !qUp){
                qUp = true;
                place.setPosition(place.getPosition() + .05);
            }
            if(!gamepad1.dpad_up && qUp){
                qUp = false;
            }

            if(gamepad1.dpad_down && !qDown){
                qDown = true;
                place.setPosition(place.getPosition() - .05);
            }
            if(!gamepad1.dpad_down && qDown){
                qDown = false;
            }


            telemetry.addData("Current Servo: ", servoNames[count]);
            telemetry.addData("Current Position: ", place.getPosition());
            telemetry.update();



        }
    }
}
