package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="XXX - Encoder Testing", group = "default")
public class EnoderTesting extends LinearOpMode {

    private static DcMotor spoolMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        spoolMotor = hardwareMap.dcMotor.get("spool");
        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();



        while(!isStopRequested()) {
            spoolMotor.setTargetPosition(800);
//            spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spoolMotor.setTargetPosition(0);
            spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (spoolMotor.isBusy()) {
                telemetry.addData("pos: ", spoolMotor.getCurrentPosition());
                telemetry.update();
            }

        }

    }



}
