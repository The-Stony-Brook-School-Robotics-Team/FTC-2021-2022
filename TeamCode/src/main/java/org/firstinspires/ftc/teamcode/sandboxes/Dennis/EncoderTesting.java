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
    private boolean pA, pB, pX, pY = false;

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
        telemetry.update();
        waitForStart();

        telemetry.clearAll();

        // Runtime
        while(!isStopRequested()) {
            // LF
            if(gamepad1.a && !pA) {
                lf.setPower(0.5);
                pA = true;
            } else if(!gamepad1.a && pA) {
                lf.setPower(0);
                pA = false;
            }
            // RF
            if(gamepad1.b && !pB) {
                rf.setPower(0.5);
                pB = true;
            } else if(!gamepad1.b && pB) {
                rf.setPower(0);
                pB = false;
            }
            // LB
            if(gamepad1.x && !pX) {
                lb.setPower(0.5);
                pX = true;
            } else if(!gamepad1.x && pX) {
                lb.setPower(0);
                pX = false;
            }
            // RB
            if(gamepad1.y && !pY) {
                rb.setPower(0.5);
                pY = true;
            } else if(!gamepad1.y && pY) {
                rb.setPower(0);
                pY = false;
            }
            telemetry.addData("lf: ", lf.getPower());
            telemetry.addData("rf: ", rf.getPower());
            telemetry.addData("lb: ", lb.getPower());
            telemetry.addData("rb: ", rb.getPower());

            telemetry.addData("lf velo: ", lf.getVelocity());
            telemetry.addData("rf velo: ", rf.getVelocity());
            telemetry.addData("lb velo: ", lb.getVelocity());
            telemetry.addData("rb velo: ", rb.getVelocity());

            telemetry.addData("left odo: ", leftodom.getCurrentPosition());
            telemetry.addData("right odo: ", rightodom.getCurrentPosition());
            telemetry.addData("center odo: ", centerodom.getCurrentPosition());

            telemetry.update();
        }

    }
}
