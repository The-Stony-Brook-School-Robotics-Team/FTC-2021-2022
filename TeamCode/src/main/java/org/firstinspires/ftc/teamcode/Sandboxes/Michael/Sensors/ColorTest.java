package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

@TeleOp(name = "Michael Color Test")
public class ColorTest extends LinearOpMode
{
    public NormalizedColorSensor color;
    float gain = 10;

    @Override
    public void runOpMode() throws InterruptedException {

        color = hardwareMap.get(NormalizedColorSensor.class, "color");

        waitForStart();

        while (opModeIsActive()) {
            color.setGain(gain);

            if(gamepad1.a){gain+=.005;}
            else if(gamepad1.b){gain-=.005;}
            else if(gamepad1.dpad_up){gain+=1;}
            else if(gamepad1.dpad_down){gain-=1;}

            telemetry.addData("Gain: ", color.getGain());
            telemetry.addData("Red: ", color.getNormalizedColors().red);
            telemetry.addData("Blue: ", color.getNormalizedColors().blue);
            telemetry.addData("Green: ", color.getNormalizedColors().green);
            telemetry.addData("Alpha: ", color.getNormalizedColors().alpha);
            telemetry.update();
        }
    }
}
