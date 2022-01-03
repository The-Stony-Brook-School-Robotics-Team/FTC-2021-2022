package org.firstinspires.ftc.teamcode.sandboxes.Michael.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
