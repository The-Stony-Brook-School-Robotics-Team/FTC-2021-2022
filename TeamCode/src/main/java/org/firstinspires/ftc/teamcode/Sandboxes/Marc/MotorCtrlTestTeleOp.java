package org.firstinspires.ftc.teamcode.Sandboxes.Marc;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController;
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

    FtcDashboard dashboard;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry);
        motorCtrls = new MotorEncoderController(hardwareMap,telemetry);
        dashboard = FtcDashboard.getInstance();



    }

    @Override
    public void loop() {

        telemetry.addData("L ODOM",motorCtrls.getLOdomValSoft());
        telemetry.addData("R ODOM",motorCtrls.getROdomValSoft());
        telemetry.addData("B ODOM",motorCtrls.getBOdomValSoft());
        telemetry.addData("L ODOM inch",MotorEncoderController.convertOdomTickToRobotInches(motorCtrls.getLOdomValSoft()));
        telemetry.addData("R ODOM inch",MotorEncoderController.convertOdomTickToRobotInches(motorCtrls.getROdomValSoft()));
        telemetry.addData("B ODOM inch",MotorEncoderController.convertOdomTickToRobotInches(motorCtrls.getBOdomValSoft()));
        telemetry.addData("Lpow",motorCtrls.LF().getPower());
        telemetry.addData("Rpow",motorCtrls.RF().getPower());
        telemetry.addData("powerRatio",motorCtrls.LF().getPower()/motorCtrls.RF().getPower());
        telemetry.addData("odomDiff",motorCtrls.getROdomValSoft() - motorCtrls.getLOdomValSoft());
        //telemetry.addData("xpos",motorCtrls.getPosition().getX());
        //telemetry.addData("ypos",motorCtrls.getPosition().getY());
       // telemetry.addData("hpos",Math.toDegrees(motorCtrls.getPosition().getHeading()));
        telemetry.update();

        if (gamepad1.x && !qX) {
            qX = true;
            TelemetryPacket packet = new TelemetryPacket();
            Pose2d currentPos = motorCtrls.getPosition();
            Canvas field = packet.fieldOverlay();
            DashboardUtil.drawRobot(field,new com.acmerobotics.roadrunner.geometry.Pose2d(currentPos.getX(),currentPos.getY(),currentPos.getHeading()));
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
            packet.put("xpos",motorCtrls.getPosition().getX());
            packet.put("ypos",motorCtrls.getPosition().getY());
            packet.put("hpos",Math.toDegrees(motorCtrls.getPosition().getHeading()));

            dashboard.sendTelemetryPacket(packet);
        }
        else if (!gamepad1.x && qX) {
            qX = false;
        }




        /*if (gamepad1.a && !qA) {
            qA = true;
            motorCtrls.resetSoftMotorEncoders();
        }
        else if (!gamepad1.a && qA) {
            qA = false;
        }*/
        if (gamepad1.a && !qA) {
            qA = true;
            motorCtrls.resetPosition();
        }
        else if (!gamepad1.a && qA) {
            qA = false;
        }


        if (gamepad1.b && !qB) {
            qB = true;
            motorCtrls.resetSoftOdom();
        }
        else if (!gamepad1.b && qB) {
            qB = false;
        }


        motorCtrls.LF().setPower(0.6*(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
        motorCtrls.RF().setPower(0.6*(-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
        motorCtrls.LB().setPower(0.6*(-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        motorCtrls.RB().setPower(0.6*(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));

    }
}
