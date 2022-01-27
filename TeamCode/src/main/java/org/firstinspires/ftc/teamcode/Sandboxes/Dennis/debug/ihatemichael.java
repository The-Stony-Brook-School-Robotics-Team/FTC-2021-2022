package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.debug;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="AUHGEUWSHG you -m -uwu")
public class ihatemichael extends OpMode {
    private Servo servo;
    private Rev2mDistanceSensor distanceSensor;

    private boolean pUp = false, pDown = false;
    private double openPosition = 0.35; // 0.5
    private double closedPosition = 0.05;
    private double threshold = 70;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "vt");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "2m");


        servo.setPosition(closedPosition);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up && !pUp)
        {
            pUp = true;
            servo.setPosition(servo.getPosition() + 0.05);

        }
        if(pUp && !gamepad1.dpad_up)
        {
            pUp = false;
        }
        if(gamepad1.dpad_down && !pDown)
        {
            pDown = true;
            servo.setPosition(servo.getPosition() - 0.05);
        }
        if(pDown && !gamepad1.dpad_down)
        {
            pDown = false;
        }

        if(gamepad1.a){
            servo.setPosition(openPosition);
        }
        if(gamepad1.b){
            servo.setPosition(closedPosition);
        }
        if(distanceSensor.getDistance(DistanceUnit.MM) < 50){
            servo.setPosition(openPosition);
            try {
                    Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            servo.setPosition(closedPosition);
        }



        telemetry.addData("position: ", servo.getPosition());
        telemetry.addData("distance: ", distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}