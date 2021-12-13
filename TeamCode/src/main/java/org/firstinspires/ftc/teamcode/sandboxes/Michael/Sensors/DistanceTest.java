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
    public Rev2mDistanceSensor rev;

    @Override
    public void runOpMode() throws InterruptedException {

        rev = hardwareMap.get(Rev2mDistanceSensor.class, "2m");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Distance in MM: ", rev.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}
