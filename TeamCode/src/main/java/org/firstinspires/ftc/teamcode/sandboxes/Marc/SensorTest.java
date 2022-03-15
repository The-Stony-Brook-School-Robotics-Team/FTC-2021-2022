package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SensorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DistanceSensor rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bd");
       // DistanceSensor sensorRange;
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {

            long startTime = System.nanoTime();
            double hehe = rangeSensor.getDistance(DistanceUnit.MM);
            double deltaTime = System.nanoTime()-startTime;
            Log.d("SensingController","Time delta: " + deltaTime/1000000 + " ms, " + deltaTime/1000000000 + "s");
            Log.d("SensingController","Distance: " + hehe);

/*
            try {telemetry.addData("raw ultrasonic", rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical", rangeSensor.rawOptical());
            telemetry.addData("Sensor cm",rangeSensor.getDistance(DistanceUnit.CM));}
            catch (Exception e)
            {
                Log.d("SensorTest","error caught");
            }
            finally {
                telemetry.update();
            }*/
        }
    }
}
