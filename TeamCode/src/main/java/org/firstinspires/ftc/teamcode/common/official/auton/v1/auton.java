package org.firstinspires.ftc.teamcode.common.official.auton.v1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.common.official.auton.v1.movement.DriveTrain;

@Autonomous(name="A - v1 auton")
public class auton extends OpMode {

    private DcMotorEx lf, rf, lb, rb;
    private DriveTrain drive;

    @Override
    public void init() {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        drive = new DriveTrain(lf, rf, lb, rb);
    }

    @Override
    public void loop() {
        drive.move(12);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}
