package org.firstinspires.ftc.teamcode.Sandboxes.Marc;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController;

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

    @Override
    public void init() {
        motorCtrls = new MotorEncoderController(hardwareMap,telemetry);
        //dashboard = FtcDashboard.getInstance();
        //telemetry = new MultipleTelemetry(telemetry);


    }

    @Override
    public void loop() {

        telemetry.addData("L ODOM",motorCtrls.getLOdomValSoft());
        telemetry.addData("R ODOM",motorCtrls.getROdomValSoft());
        telemetry.addData("B ODOM",motorCtrls.getBOdomValSoft());
        telemetry.addData("Lpow",motorCtrls.LF().getPower());
        telemetry.addData("Rpow",motorCtrls.RF().getPower());
        telemetry.addData("powerRatio",motorCtrls.LF().getPower()/motorCtrls.RF().getPower());
        telemetry.addData("odomDiff",motorCtrls.getROdomValSoft() - motorCtrls.getLOdomValSoft());
        telemetry.addData("TO Travel",30000);

        telemetry.update();


        /*if (gamepad1.a && !qA) {
            qA = true;
            motorCtrls.resetSoftMotorEncoders();
        }
        else if (!gamepad1.a && qA) {
            qA = false;
        }*/
        if (gamepad1.a && !qA) {
            qA = true;
            System.out.println("start" + System.nanoTime());
            motorCtrls.goForwardDistPID(72);
            System.out.println("start" + System.nanoTime());
        }
        else if (!gamepad1.a && qA) {
            qA = false;
        }
        if (gamepad1.b && !qB) {
            qB = true;
            motorCtrls.goBackwardDist(2);
        }
        else if (!gamepad1.b && qB) {
            qB = false;
        }
        if (gamepad1.x && !qX) {
            qX = true;
            motorCtrls.strafeLeftDist(2);
        }
        else if (!gamepad1.x && qX) {
            qX = false;
        }
        if (gamepad1.y && !qY) {
            qY = true;
            motorCtrls.strafeRightDist(2);
        }
        else if (!gamepad1.y && qY) {
            qY = false;
        }



        motorCtrls.LF().setPower(0.6*(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));
        motorCtrls.RF().setPower(0.6*(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
        motorCtrls.LB().setPower(0.6*(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
        motorCtrls.RB().setPower(0.6*(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));

    }
}
