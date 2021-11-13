
package org.firstinspires.ftc.teamcode.sandboxes.Max;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="MaxRoadRunnerStudy", group="drive")
public class MaxRoadRunnerStudying extends OpMode {

    private static DcMotor lf;
    private static DcMotor rf;
    private static DcMotor lb;
    private static DcMotor rb;
    private MotorEx LeftEncoder;
    private MotorEx RightEncoder;
    private MotorEx CentralEncoder;
    private SampleMecanumDrive SampleMecanumDrive;
    TelemetryPacket TelemetryPacket = new TelemetryPacket();
    private FtcDashboard FTCDashboard;
    Canvas Canvas = TelemetryPacket.fieldOverlay();

    @Override
    public void init(){
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "leftodom");
        lb = hardwareMap.get(DcMotor.class, "backodom");
        rb = hardwareMap.get(DcMotor.class, "rightodom");
        LeftEncoder = new MotorEx(hardwareMap, "leftodom");
        CentralEncoder = new MotorEx(hardwareMap, "backodom");
        RightEncoder = new MotorEx(hardwareMap, "rightodom");
        SampleMecanumDrive = new SampleMecanumDrive(hardwareMap);
        SampleMecanumDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // FTCDashboard = new FtcDashboard.getInstance();

    }

    @Override
    public void loop(){


        SampleMecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        -0.3*gamepad1.left_stick_y,
                        -0.3*gamepad1.left_stick_x,
                        -0.3*gamepad1.right_stick_x
                )
        );




    }



}
