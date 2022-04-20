package org.firstinspires.ftc.teamcode.common.official.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="A - Motor Direction Debugger")
public class MotorDirectionDebugger extends LinearOpMode {

    private DcMotorEx lf, rf, lb, rb;
    private DcMotorEx selectedMotor;

    private boolean pressingA = false;
    private boolean pressingX = false;
    private boolean pressingB = false;
    private boolean pressingRight = false;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        selectedMotor = lf;

        while(!isStopRequested()) {
            // switch
            if(gamepad1.dpad_right && !pressingRight) {
                if(selectedMotor == null) {
                    selectedMotor = lf;
                }
                if(selectedMotor == lf) {
                    selectedMotor = rf;
                }
                if(selectedMotor == rf) {
                    selectedMotor = lb;
                }
                if(selectedMotor == lb) {
                    selectedMotor = rb;
                }
                if(selectedMotor == rb) {
                    selectedMotor = lf;
                }
                pressingRight = true;
            } else if(!gamepad1.dpad_right && pressingRight) {
                pressingRight = false;
            }

            if(gamepad1.x && !pressingX) {
                selectedMotor.setTargetPosition(selectedMotor.getCurrentPosition() + 500);
                selectedMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pressingX = true;
            } else if(!gamepad1.x && pressingX) {
                pressingX = false;
            }

            // b
            if(gamepad1.b && !pressingB) {
                selectedMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                selectedMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                pressingB = true;
            } else if(!gamepad1.b && pressingB) {
                pressingB = false;
            }

            // a
            if(gamepad1.a && !pressingA) {
                if(selectedMotor.getDirection() == DcMotorSimple.Direction.REVERSE) {
                    selectedMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                } else {
                    selectedMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                pressingA = true;
            } else if(!gamepad1.a && pressingA) {
                pressingA = false;
            }

            selectedMotor.setPower(gamepad1.left_stick_y);
            telemetry.addData("Motor: ", selectedMotor.getDeviceName());
            telemetry.addData("Power: ", selectedMotor.getPower());
            telemetry.addData("Direction: ", selectedMotor.getDirection());
            telemetry.addData("Position: ", selectedMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
