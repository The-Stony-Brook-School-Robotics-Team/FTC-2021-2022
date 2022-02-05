package org.firstinspires.ftc.teamcode.sandboxes.Michael.Sensors;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@TeleOp(name = "Distance Test")
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
