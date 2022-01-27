package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="fuck you -m -uwu")
public class ihatemichael extends OpMode {
    private Servo servo;
    private boolean pUp = false, pDown = false;
    private double openPosition = 0.5;
    private double closedPosition = 0.05;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "vt");
        servo.setPosition(openPosition);
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



        telemetry.addData("position: ", servo.getPosition());
        telemetry.update();
    }
}