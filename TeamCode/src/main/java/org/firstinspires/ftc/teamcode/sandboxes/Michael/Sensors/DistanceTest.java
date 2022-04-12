package org.firstinspires.ftc.teamcode.sandboxes.Michael.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Distance Test")
public class DistanceTest extends LinearOpMode
{

    byte[] redCache;
    byte[] blueCache;
    ModernRoboticsI2cRangeSensor redDevice;
    ModernRoboticsI2cRangeSensor blueDevice;
    I2cDeviceSynch redSynch;
    I2cDeviceSynch blueSynch;


    @Override
    public void runOpMode() throws InterruptedException {

        redDevice = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rd");
        blueDevice = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bd");
        blueDevice.setI2cAddress(I2cAddr.create8bit(0x24));

        waitForStart();

        while (opModeIsActive()) {


            telemetry.addData("red Distance", redDevice.getDistance(DistanceUnit.MM));
            telemetry.addData("blue Distance", blueDevice.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
