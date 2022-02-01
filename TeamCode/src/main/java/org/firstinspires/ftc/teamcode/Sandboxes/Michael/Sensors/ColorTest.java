package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Michael Color Test")
public class ColorTest extends LinearOpMode
{
    public ColorRangeSensor redColor;
    public ColorRangeSensor blueColor;
    public Servo dumper;
    float gain = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        redColor = hardwareMap.get(ColorRangeSensor.class, "rc");
        blueColor = hardwareMap.get(ColorRangeSensor.class, "bc");
        dumper = hardwareMap.get(Servo.class, "du");


        waitForStart();


        while (opModeIsActive()) {
            if(blueColor.alpha() > 160){
                dumper.setPosition(.55);
            }
            if(gamepad1.a){
                dumper.setPosition(.2);
            }
            if(gamepad1.b){
                dumper.setPosition(.55);
            }
            if(gamepad1.x){
                dumper.setPosition(0);
            }
            telemetry.addData("position: ", dumper.getPosition());
            telemetry.update();
        }
    }
}
