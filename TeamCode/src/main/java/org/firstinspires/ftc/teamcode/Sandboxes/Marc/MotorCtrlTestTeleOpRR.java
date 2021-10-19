package org.firstinspires.ftc.teamcode.Sandboxes.Marc;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController;
import org.firstinspires.ftc.teamcode.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp

public class MotorCtrlTestTeleOpRR extends OpMode {
    //MotorEncoderController motorCtrls;
    private boolean qA = false;
    private boolean qB = false;
    private boolean qX = false;
    private boolean qY = false;

    SampleMecanumDrive drive;
    public static final double TRACKWIDTH = 14.31;
    public static final double CENTER_WHEEL_OFFSET = 0.477;
    public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double TICKS_TO_INCHES = TICKS_PER_REV*WHEEL_DIAMETER*Math.PI;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    T265Controller camCtrl;
    FtcDashboard dashboard;
        RobotState state = RobotState.STOPPED;

        double iniH;
    double iniX;
    double iniY;


    @Override
    public void init() {
        msStuckDetectInit = 1000000;
        msStuckDetectLoop = 1000000;
        msStuckDetectStop = 1000000;
        msStuckDetectStart = 1000000;
        telemetry = new MultipleTelemetry(telemetry);
       // motorCtrls = new MotorEncoderController(hardwareMap,telemetry);
        dashboard = FtcDashboard.getInstance();
        camCtrl = new T265Controller(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(camCtrl.getIntelPos());

    }

    @Override
    public void loop() {
        Pose2d currentPos = camCtrl.getIntelPos();
        drive.setPoseEstimate(currentPos);
        telemetry.addData("x",currentPos.getX());
        telemetry.addData("y",currentPos.getY());
        telemetry.addData("h",currentPos.getHeading());
         telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();
            DashboardUtil.drawRobot(field,currentPos);
            packet.put("xpos",currentPos.getX());
            packet.put("ypos",currentPos.getY());
            packet.put("hpos",currentPos.getHeading());

            dashboard.sendTelemetryPacket(packet);







        if (gamepad1.a && !qA) {
            qA = true;
            state = RobotState.TURNR;
            iniH = camCtrl.getIntelPos().getHeading();
            return;

        }
        else if (!gamepad1.a && qA) {
            qA = false;

        }

        if (gamepad1.y && !qY) {
            qY = true;
            state = RobotState.STRAFEL;
            iniY = camCtrl.getIntelPos().getY();
            return;

        }
        else if (!gamepad1.y && qY) {
            qY = false;

        }


        if (gamepad1.b && !qB) {
            qB = true;
            iniX = camCtrl.getIntelPos().getX();
            state = RobotState.FORWARD;
        }
        else if (!gamepad1.b && qB) {
            qB = false;
        }

        doStateCommands();

    }

    @Override
    public void stop() {
        super.stop();
        camCtrl.shutDown();
    }
    public void doStateCommands()
    {
        Pose2d currentPos = camCtrl.getIntelPos();
        switch(state) {
            case TURNR:
                drive.turn(Math.PI/2);
               /* if(currentPos.getHeading() <= iniH + Math.PI/2)
                {
                    //motorCtrls.turnRightPower(0.3);
                }
                else {//motorCtrls.stopRobot();
                */state = RobotState.STOPPED;

            case FORWARD:
                drive.followTrajectory(drive.trajectoryBuilder(currentPos).forward(24).build());
                state = RobotState.STOPPED;
            case STRAFEL:
                drive.followTrajectory(drive.trajectoryBuilder(currentPos).strafeLeft(24).build());
                state = RobotState.STOPPED;
        }
    }
}

enum RobotState1 {
    STOPPED,
    GAMEPAD,
    FORWARD,
    BACKWARD,
    STRAFER,
    STRAFEL,
    TURNR,
    TURNL,
    AUTO

}