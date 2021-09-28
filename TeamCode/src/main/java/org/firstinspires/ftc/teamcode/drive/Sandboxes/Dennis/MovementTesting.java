package org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.EnhancedMovement;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.annotations.Testing;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.exceptions.MovementException;
import org.firstinspires.ftc.teamcode.drive.Sandboxes.Dennis.EnhancedMovement.geometry.Point;

import java.util.HashMap;

@Testing
@Autonomous(name="Dennis Auton", group="drive")
public class MovementTesting extends LinearOpMode {

    private static SampleMecanumDrive drive;

    // Imu
    private BNO055IMU imu = null;

    // Motors
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private HashMap<String, DcMotor> motors = new HashMap<String, DcMotor>();

    // Used in location tracking
    Pose2d currentPos = new Pose2d(0, 0);

    @Override
    public void runOpMode() throws InterruptedException
    {
        // RoadRunner Positions
        Pose2d currentPos = new Pose2d(0, 0);
        // IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Motors
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
        motors.put("lf", lf);
        motors.put("rf", rf);
        motors.put("lb", lb);
        motors.put("rb", rb);
        // drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(currentPos);


        // Pre Loop
        waitForStart();

        // Code
        //currentPos = Movement.forward(currentPos, drive, inches);
        //currentPos = Movement.strafeLeft(currentPos, drive, inches);
        //currentPos = Movement.back(currentPos, drive, inches);
        //currentPos = Movement.strafeRight(currentPos, drive, inches);

        while(!isStopRequested())
        {

        }


    }
}
