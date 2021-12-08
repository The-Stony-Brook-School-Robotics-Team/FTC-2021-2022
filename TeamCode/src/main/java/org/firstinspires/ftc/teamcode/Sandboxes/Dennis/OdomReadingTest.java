package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="AA - Odom Reading Software")
public class OdomReadingTest extends LinearOpMode {


    private static DcMotorEx left, right;

    @Override
    public void runOpMode() throws InterruptedException {
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        while(!isStopRequested()) {
            telemetry.addData("left: ", left.getCurrentPosition());
            telemetry.addData("right: ", right.getCurrentPosition());
            telemetry.update();
        }

    }


}
