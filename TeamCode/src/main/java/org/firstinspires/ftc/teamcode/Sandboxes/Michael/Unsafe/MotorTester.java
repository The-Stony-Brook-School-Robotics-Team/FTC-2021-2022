package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="BLINK BLINK BLINK LIGHT GO BRRR", group="Linear Opmode")
public class MotorTester extends LinearOpMode {
    private Servo mini;

    public void runOpMode() throws InterruptedException {
        mini = hardwareMap.get(Servo.class, "vout");
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                mini.setPosition(1);
                Thread.sleep(200);
                mini.setPosition(0);
                Thread.sleep(200);
            }
           if(gamepad1.b){
               for(double i = mini.getPosition(); i < 1; i+=.001){
                   mini.setPosition(i);
               }
               for(double i = mini.getPosition(); i > 0; i-=.001){
                   mini.setPosition(i);
               }
           }
            if(gamepad1.y){
                mini.setPosition(1);
                Thread.sleep(10);
                mini.setPosition(0);
                Thread.sleep(10);
            }

        }


    }
}
