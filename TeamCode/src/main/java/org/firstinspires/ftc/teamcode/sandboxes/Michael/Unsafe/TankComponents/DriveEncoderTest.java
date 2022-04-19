package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "drive encoder test")
public class DriveEncoderTest extends OpMode {
    //private DriveController driveController;
    DcMotorEx lf, rf, lb, rb;
    @Override
    public void init() {
      lf = hardwareMap.get(DcMotorEx.class, "lf");
      rf = hardwareMap.get(DcMotorEx.class, "rf");
      lb = hardwareMap.get(DcMotorEx.class, "lb");
      rb = hardwareMap.get(DcMotorEx.class, "rb");

      lf.setDirection(DcMotorSimple.Direction.REVERSE);
      lb.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void start(){



    }

    @Override
    public void loop() {
        if(gamepad1.a){
            lf.setPower(.3);
            rf.setPower(.3);
            lb.setPower(.3);
            rb.setPower(.3);
        }
        else{
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }

    }

}
