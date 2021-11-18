package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ColorTest extends OpMode {
   Servo colorstrip;
    @Override
    public void init() {
        colorstrip = hardwareMap.get(Servo.class,"colorstrip");
    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            colorstrip.setPosition(0.02525);
        }
        if(gamepad1.b) {
            colorstrip.setPosition(0.4);
        }
        if(gamepad1.x) {
            colorstrip.setPosition(0.6);
        }
        if(gamepad1.y) {
            colorstrip.setPosition(0.7475);
        }
    }
}
