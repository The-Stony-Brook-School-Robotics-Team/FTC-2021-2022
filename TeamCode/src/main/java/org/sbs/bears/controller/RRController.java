package org.sbs.bears.controller;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.StandardDrive;
import org.sbs.bears.util.SubsystemController;

public class RRController extends SubsystemController {
    StandardDrive drive;
    public RRController(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
        this.drive = new StandardDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
    }

    @Override
    public boolean shutDown() {
        return true;
    }
}
