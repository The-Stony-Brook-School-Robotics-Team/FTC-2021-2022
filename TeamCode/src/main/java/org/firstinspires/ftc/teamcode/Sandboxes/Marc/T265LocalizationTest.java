package org.firstinspires.ftc.teamcode.Sandboxes.Marc;


//import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.util.T265Controller;
import org.firstinspires.ftc.teamcode.common.RobotState;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.T265Localizer;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
@TeleOp(name = "T - T265LocalizationTest", group = "drive")
    public class T265LocalizationTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            RevBlinkinLedDriver colorstrip2;
            colorstrip2 = hardwareMap.get(RevBlinkinLedDriver.class,"colorstrip");
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            SampleMecanumDrive.isUsingT265 = true;
            org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            waitForStart();
            colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

            while (!isStopRequested()) {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();

                if(gamepad1.a) {
                    colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    drive = null;
                    T265Controller.shutDown();
                    drive = new SampleMecanumDrive(hardwareMap);
                    colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    drive.setLocalizer(new T265Localizer(hardwareMap));
                    colorstrip2.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                }

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
            }
        }
    }
