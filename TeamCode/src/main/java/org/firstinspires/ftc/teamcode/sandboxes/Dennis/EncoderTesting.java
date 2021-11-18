package org.firstinspires.ftc.teamcode.sandboxes.Dennis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "default", name="U - EncoderTesting")
public class EncoderTesting extends LinearOpMode {

    private DcMotorEx leftodom, rightodom, centerodom;
    private DcMotorEx lf, rf, rb, lb;
    private boolean pA, pB, pX, pY, pUp, pLD = false;
    private boolean tUp = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftodom = hardwareMap.get(DcMotorEx.class, "leftodom");
        rightodom = hardwareMap.get(DcMotorEx.class, "rightodom");
        centerodom = hardwareMap.get(DcMotorEx.class, "centerodom");

        rightodom.setDirection(DcMotorSimple.Direction.REVERSE);
        centerodom.setDirection(DcMotorSimple.Direction.REVERSE);


        // Pre-Runtime
        telemetry.addLine("Initialized");

        telemetry.addData("lf info: ", lf.getConnectionInfo());
        telemetry.addData("rf info: ", rf.getConnectionInfo());
        telemetry.addData("lb info: ", lb.getConnectionInfo());
        telemetry.addData("rb info: ", rb.getConnectionInfo());

        telemetry.update();
        waitForStart();

        telemetry.clearAll();

        // Runtime
        while(!isStopRequested()) {

            // LF
            if(gamepad1.a && !pA) {
                lf.setPower(0.9);
                pA = true;
            } else if(!gamepad1.a && pA) {
                lf.setPower(0);
                pA = false;
            }

            // RF
            if(gamepad1.b && !pB) {
                rf.setPower(0.9);
                pB = true;
            } else if(!gamepad1.b && pB) {
                rf.setPower(0);
                pB = false;
            }

            // LB
            if(gamepad1.x && !pX) {
                lb.setPower(0.9);
                pX = true;
            } else if(!gamepad1.x && pX) {
                lb.setPower(0);
                pX = false;
            }

            // RB
            if(gamepad1.y && !pY) {
                rb.setPower(0.9);
                pY = true;
            } else if(!gamepad1.y && pY) {
                rb.setPower(0);
                pY = false;
            }

            // Odom Reset
            if(gamepad1.dpad_left && !pLD) {
                pLD = true;
            } else if(!gamepad1.dpad_left && pLD) {
                leftodom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightodom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                centerodom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pLD = false;
            }

            // Toggle Odom Telemetry
            if(gamepad1.dpad_up && !pUp) {
                pUp = true;
            } else if(!gamepad1.dpad_up && pUp) {
                telemetry.clearAll();
                tUp = !tUp;
                pUp = false;
            }

            // Weighted Driving
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);
            // End


            if(!tUp) {
                telemetry.addLine("-------------------------------------");
                telemetry.addData("lf: ", lf.getPower());
                telemetry.addData("rf: ", rf.getPower());
                telemetry.addData("lb: ", lb.getPower());
                telemetry.addData("rb: ", rb.getPower());
                telemetry.addLine("-------------------------------------");
                telemetry.addData("lf velo: ", lf.getVelocity());
                telemetry.addData("rf velo: ", rf.getVelocity());
                telemetry.addData("lb velo: ", lb.getVelocity());
                telemetry.addData("rb velo: ", rb.getVelocity());
                telemetry.addLine("-------------------------------------");
            }

            if(tUp) {
                telemetry.addLine("-------------------------------------");
                telemetry.addData("left odo: ", leftodom.getCurrentPosition());
                telemetry.addData("right odo: ", rightodom.getCurrentPosition());
                telemetry.addData("center odo: ", centerodom.getCurrentPosition());
                telemetry.addLine("-------------------------------------");
            }

            telemetry.update();
        }

    }
}
