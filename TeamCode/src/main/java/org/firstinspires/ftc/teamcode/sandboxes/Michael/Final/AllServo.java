package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="B- ALL SERVOS", group="Linear Opmode")
public class AllServo extends LinearOpMode {
    private Servo bucket;
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


        Servo[] servos = {bucket, redIntake, blueIntake, blueSweeper, verticalServo, dumper};
        place = servos[0];
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a && !qA){
                qA = true;
                try {
                    count++;
                }catch (Exception e){
                    count = 0;
                }
                place = servos[0];
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
            if(!gamepad1.dpad_up && qDown){
                qDown = false;
            }


            telemetry.addData("Current Servo: ", place);
            telemetry.update();



        }
    }
}
