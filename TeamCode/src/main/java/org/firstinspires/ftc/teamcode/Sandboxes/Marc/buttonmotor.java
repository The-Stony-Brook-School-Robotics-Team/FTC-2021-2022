package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import static org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController.motorNames;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp
public class buttonmotor extends LinearOpMode {
    DcMotor[] motors = new DcMotor[4];


    @Override
    public void runOpMode() throws InterruptedException {


        for (int i = 0; i < 4; i++) {
            motors[i] = (hardwareMap.get(DcMotor.class, motorNames[i]));
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

        while(opModeIsActive()) {


            if(gamepad1.a) {
                motors[0].setPower(1);
                continue;
            }
            if(gamepad1.b) {
                motors[1].setPower(1);
                continue;
            }
            if(gamepad1.y) {
                motors[2].setPower(1);                continue;
            }
            if(gamepad1.x) {
                motors[3].setPower(1);                continue;
            }
            stopMotors();



        }

    }
    public void forwardPow(double pow) {
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(pow);
        }
    }
    public void stopMotors()
    {
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(0);
        }
    }

}
