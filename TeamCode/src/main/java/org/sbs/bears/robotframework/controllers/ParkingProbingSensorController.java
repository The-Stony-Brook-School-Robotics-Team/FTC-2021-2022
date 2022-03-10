package org.sbs.bears.robotframework.controllers;

import android.util.Log;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ParkingProbingSensorController {
    Rev2mDistanceSensor sensorBlue;
    Rev2mDistanceSensor sensorRed;
    public ParkingProbingSensorController(HardwareMap hardwareMap, Telemetry telemetry){
        sensorBlue = hardwareMap.get(Rev2mDistanceSensor.class, "bdist");
        sensorRed = hardwareMap.get(Rev2mDistanceSensor.class, "rdist");
        Log.d("ParkingProberController","All sensors initialized");
    }
    public double getDistBlue()
    {
        return sensorBlue.getDistance(DistanceUnit.INCH);
    }
    public double getDistRed()
    {
        return sensorRed.getDistance(DistanceUnit.INCH);
    }
    public boolean isOpeningAvailableAuton(boolean qBlue, double distToTarget)
    {
        if(qBlue)
        {
            double dist = sensorBlue.getDistance(DistanceUnit.INCH);
            Log.d("ParkingProberController","Blue Distance: " + dist);
            return dist  >= distToTarget;
        }
        else
        {
            double dist = sensorRed.getDistance(DistanceUnit.INCH);
            Log.d("ParkingProberController","Red Distance: " + dist);
            return dist  >= distToTarget;
        }
    }
}
