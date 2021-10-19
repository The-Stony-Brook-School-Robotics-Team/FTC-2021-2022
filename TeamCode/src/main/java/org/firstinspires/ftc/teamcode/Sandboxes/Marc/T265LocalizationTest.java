package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import static org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController.motorNames;
import static org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController.odomNames;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp
@Config
public class T265LocalizationTest extends LinearOpMode {
    DcMotor[] motors = new DcMotor[4];
    DcMotor[] odoms = new DcMotor[3];
    boolean qA,qB,qX,qY;
    RobotState state = RobotState.STOPPED;
    Pose2d currentPos;
    public static int DIST = 72;

    double iniX;
    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
msStuckDetectLoop = 500000000;
msStuckDetectStart = 500000000;
msStuckDetectInit = 500000000;
msStuckDetectStop = 500000000;
         dashboard = FtcDashboard.getInstance();
        Log.d("265localizer","starting up camera");
        T265Controller camCtrl = new T265Controller(hardwareMap,telemetry);
        for (int i = 0; i < 4; i++) {
            motors[i] = (hardwareMap.get(DcMotor.class, motorNames[i]));
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

        while(opModeIsActive()) {

            currentPos = camCtrl.getIntelPos();

            TelemetryPacket packet3 = new TelemetryPacket();
            Canvas field3 = packet3.fieldOverlay();
            DashboardUtil.drawRobot(field3,currentPos);
            packet3.put("xpos",currentPos.getX());
            packet3.put("ypos",currentPos.getY());
            packet3.put("hpos",currentPos.getHeading());
            packet3.put("state",state);

        telemetry.addData("state",state);
        telemetry.update();

            if(gamepad1.a && !qA) {
                qA = true;
                iniX = currentPos.getX();
                while (Math.abs(currentPos.getX() - iniX) < DIST) {
                    forwardPow(0.3);
                    TelemetryPacket pack = new TelemetryPacket();
                    pack.put("xpos",currentPos.getX());
                    pack.put("iniX",iniX);
                    pack.put("state",state);
                    dashboard.sendTelemetryPacket(pack);
                }
                stopMotors();
                continue;
            }
            else if (!gamepad1.a && qA) {
                qA = false;
            }
            if(gamepad1.b && !qB) {
                qB = true;
                state = (state.equals(RobotState.STOPPED)) ? RobotState.GAMEPAD : RobotState.STOPPED;
                continue;
            }
            else if (!gamepad1.b && qB) {
                qB = false;
            }
            if(gamepad1.y && !qY) {
                qY = true;
                T265Controller.intelCam.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0,0,new Rotation2d(0)));
                continue;
            }
            else if (!gamepad1.y && qY) {
                qY = false;
            }
            motors[3].setPower(0.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            motors[2].setPower(0.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            motors[1].setPower(0.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            motors[0].setPower(0.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            //doStateCommands();


        }
        Log.d("265localizer","shutting down camera");
        new Thread(() -> {camCtrl.shutDown();}).start();
        sleep(1000);
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

   /* public void doStateCommands() {
        switch (state) {
            case FORWARD:
                if (Math.abs(currentPos.getX() - iniX) < DIST) {
                    forwardPow(0.3);
                    TelemetryPacket pack = new TelemetryPacket();
                    pack.put("xpos",currentPos.getX());
                    pack.put("iniX",iniX);
                    pack.put("state",state);
                    dashboard.sendTelemetryPacket(pack);
                } else {
                    stopMotors();
                    state = RobotState.STOPPED;
                }
            case GAMEPAD:
                TelemetryPacket packet = new TelemetryPacket();
                Canvas field = packet.fieldOverlay();
                DashboardUtil.drawRobot(field,currentPos);
                packet.put("xpos",currentPos.getX());
                packet.put("ypos",currentPos.getY());
                packet.put("hpos",currentPos.getHeading());
                packet.put("state",state);

                dashboard.sendTelemetryPacket(packet);
                motors[3].setPower(0.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
                motors[2].setPower(0.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
                motors[1].setPower(0.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
                motors[0].setPower(0.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
            case STOPPED:
                TelemetryPacket packet3 = new TelemetryPacket();
                Canvas field3 = packet3.fieldOverlay();
                DashboardUtil.drawRobot(field3,currentPos);
                packet3.put("xpos",currentPos.getX());
                packet3.put("ypos",currentPos.getY());
                packet3.put("hpos",currentPos.getHeading());
                packet3.put("state",state);

                dashboard.sendTelemetryPacket(packet3);
                stopMotors();
        }
    }*/
}
