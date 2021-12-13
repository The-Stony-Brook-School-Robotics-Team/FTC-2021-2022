package org.firstinspires.ftc.teamcode.sandboxes.Michael.Final;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "Color Tape Detection")
public class ColorReading extends LinearOpMode
{
    public NormalizedColorSensor color;
    float gain = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        color.setGain(gain);


        waitForStart();

        while (opModeIsActive()) {



        }
    }
}
