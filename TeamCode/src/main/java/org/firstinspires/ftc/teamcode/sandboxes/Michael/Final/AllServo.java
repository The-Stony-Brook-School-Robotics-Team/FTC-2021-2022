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
    private Servo stopper;
    private Servo redSweeper;
    private Servo redStopper;
    private Servo redDumper;

    private boolean qA = false;
    private boolean qUp = false;
    private boolean qDown = false;
    int count = 0;

    private Servo place;





    public void runOpMode() throws InterruptedException {
        redIntake = hardwareMap.get(Servo.class, "ri");
        blueIntake = hardwareMap.get(Servo.class, "bi");
        blueSweeper = hardwareMap.get(Servo.class, "sweep");
        verticalServo = hardwareMap.get(Servo.class, "vt");
        dumper = hardwareMap.get(Servo.class, "du");
        stopper = hardwareMap.get(Servo.class, "bs");
        redSweeper = hardwareMap.get(Servo.class, "rsweep");
        redStopper = hardwareMap.get(Servo.class, "rs");
        redDumper = hardwareMap.get(Servo.class, "rdu");


        Servo[] servos = {redIntake, blueIntake, blueSweeper, verticalServo, dumper, stopper, redSweeper, redStopper, redDumper};
        String[] servoNames = {"red intake", "blue intake", "blue sweeper", "vertical servo", "blue dumper", "blue stopper", "red sweeper", "red stopper", "red dumper"};
        place = servos[0];
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a && !qA){
                qA = true;
                count++;
                if(count > servos.length - 1) count = 0;
                place = servos[count];
                place.setPosition(place.getPosition());
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
