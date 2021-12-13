package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "Color Tape Detection")
public class ColorReading extends LinearOpMode
{
    public NormalizedColorSensor color;
    int gain = 10;

    /** Array order is red, green, blue, alpha */
    final double[] RED = {.45, .13, .12, .62};
    final double[] BLUE = {.10, .36, .25, .66};
    final double[] WHITE = {.89, .86, .12, } ;


    @Override
    public void runOpMode() throws InterruptedException {

        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        color.setGain(gain);
        waitForStart();

        while (opModeIsActive()) {



        }
    }
}
