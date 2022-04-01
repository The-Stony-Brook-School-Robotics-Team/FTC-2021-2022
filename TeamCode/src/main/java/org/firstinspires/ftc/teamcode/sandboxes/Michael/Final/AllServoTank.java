package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="B - ALL SERVOS TANK", group="Linear Opmode")
public class AllServoTank extends LinearOpMode {
    private Servo dunker;

    private boolean qA = false;
    private boolean qUp = false;
    private boolean qDown = false;
    int count = 0;
    private int up = 1;
    private int down = 0;

    private Servo place;





    public void runOpMode() throws InterruptedException {
        dunker = hardwareMap.get(Servo.class, "dunk");

        Servo[] servos = {dunker};
        String[] servoNames = {"dunker"};
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
                place.setPosition(place.getPosition() + .01);
            }
            if(!gamepad1.dpad_up && qUp){
                qUp = false;
            }

            if(gamepad1.dpad_down && !qDown){
                qDown = true;
                place.setPosition(place.getPosition() - .01);
            }
            if(!gamepad1.dpad_down && qDown){
                qDown = false;
            }
            if(gamepad1.x){
                place.setPosition(up);
            }
            if(gamepad1.y){
                place.setPosition(down);
            }


            telemetry.addData("Current Servo: ", servoNames[count]);
            telemetry.addData("Current Position: ", place.getPosition());
            telemetry.addLine();
            telemetry.addData("Hit X to set to " + up + ". Hit Y to set to " + down, "");
            telemetry.update();



        }
    }
}
