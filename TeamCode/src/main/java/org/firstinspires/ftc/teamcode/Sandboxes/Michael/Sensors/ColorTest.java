package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "Michael Color Test")
public class ColorTest extends LinearOpMode
{
    public ColorRangeSensor redColor;
    public ColorRangeSensor blueColor;
    float gain = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        redColor = hardwareMap.get(ColorRangeSensor.class, "rc");
        blueColor = hardwareMap.get(ColorRangeSensor.class, "bc");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("red Red: ", redColor.red());
            telemetry.addData("red Blue: ", redColor.blue());
            telemetry.addData("red Green: ", redColor.green());
            telemetry.addData("red Alpha: ", redColor.alpha());
            telemetry.addLine();
            telemetry.addData("blue Red: ", blueColor.red());
            telemetry.addData("blue Blue: ", blueColor.blue());
            telemetry.addData("blue Green: ", blueColor.green());
            telemetry.addData("blue Alpha: ", blueColor.alpha());

            telemetry.update();
        }
    }
}
