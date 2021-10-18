package org.firstinspires.ftc.teamcode.Sandboxes.Marc;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController;
import org.firstinspires.ftc.teamcode.BearsUtil.T265Controller;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp

public class MotorCtrlTestTeleOp extends OpMode {
    MotorEncoderController motorCtrls;
    private boolean qA = false;
    private boolean qB = false;
    private boolean qX = false;
    private boolean qY = false;


    public static final double TRACKWIDTH = 14.31;
    public static final double CENTER_WHEEL_OFFSET = 0.477;
    public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double TICKS_TO_INCHES = TICKS_PER_REV*WHEEL_DIAMETER*Math.PI;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    T265Controller camCtrl;
    FtcDashboard dashboard;




    @Override
    public void init() {
        msStuckDetectInit = 1000000;
        msStuckDetectLoop = 1000000;
        msStuckDetectStop = 1000000;
        msStuckDetectStart = 1000000;
        telemetry = new MultipleTelemetry(telemetry);
      //  motorCtrls = new MotorEncoderController(hardwareMap,telemetry);
        dashboard = FtcDashboard.getInstance();
        camCtrl = new T265Controller(hardwareMap,telemetry);


    }

    @Override
    public void loop() {
        com.acmerobotics.roadrunner.geometry.Pose2d currentPos = camCtrl.getIntelPos();
        telemetry.addData("x",currentPos.getX());
        telemetry.addData("y",currentPos.getY());
        telemetry.addData("h",currentPos.getHeading());
         telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas field = packet.fieldOverlay();
            DashboardUtil.drawRobot(field,currentPos);
            //packet.put("L ODOM",motorCtrls.getLOdomValSoft());
            //packet.put("R ODOM",motorCtrls.getROdomValSoft());
            //packet.put("B ODOM",motorCtrls.getBOdomValSoft());
            //packet.put("L ODOM inch",MotorEncoderController.convertOdomTickToRobotInches(motorCtrls.getLOdomValSoft()));
            // packet.put("R ODOM inch",MotorEncoderController.convertOdomTickToRobotInches(motorCtrls.getROdomValSoft()));
            //packet.put("B ODOM inch",MotorEncoderController.convertOdomTickToRobotInches(motorCtrls.getBOdomValSoft()));
            // packet.put("Lpow",motorCtrls.LF().getPower());
            // packet.put("Rpow",motorCtrls.RF().getPower());
            //packet.put("powerRatio",motorCtrls.LF().getPower()/motorCtrls.RF().getPower());
            // packet.put("odomDiff",motorCtrls.getROdomValSoft() - motorCtrls.getLOdomValSoft());
            // packet.put("TO Travel",30000);
            packet.put("xpos",currentPos.getX());
            packet.put("ypos",currentPos.getY());
            packet.put("hpos",currentPos.getHeading());

            dashboard.sendTelemetryPacket(packet);







        if (gamepad1.a && !qA) {
            qA = true;
            //motorCtrls.resetPosition();
        }
        else if (!gamepad1.a && qA) {
            qA = false;
        }


        if (gamepad1.b && !qB) {
            qB = true;
            //motorCtrls.resetSoftOdom();
        }
        else if (!gamepad1.b && qB) {
            qB = false;
        }



    }

    @Override
    public void stop() {
        super.stop();
        camCtrl.shutDown();
    }
}
