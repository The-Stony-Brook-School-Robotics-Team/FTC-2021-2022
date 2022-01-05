package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotObjects {

    private static HardwareMap internalHardwareMap = null;
    private static Telemetry internalTelemetry = null;


    public RobotObjects(HardwareMap hardwareMap, Telemetry telemetry) {
        internalHardwareMap = hardwareMap;
        internalTelemetry = telemetry;
    }


}
