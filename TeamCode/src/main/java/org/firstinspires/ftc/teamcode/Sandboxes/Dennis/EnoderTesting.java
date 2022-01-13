package org.firstinspires.ftc.teamcode.Sandboxes.Dennis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp(name="XXX - Encoder Testing", group = "default")
public class EnoderTesting extends LinearOpMode {

    private static DcMotor spoolMotor;

    private static double currentPower = 0;

    private static boolean stopRequested = false;

    @Override
    public void runOpMode() throws InterruptedException {
        spoolMotor = hardwareMap.dcMotor.get("spool");
        spoolMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spoolMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        powerUpdater.start();


        waitForStart();

        spoolMotor.setPower(currentPower);
        runToPosition(0);

        while(!isStopRequested()) {
            stopRequested = isStopRequested();

            runToPosition(800);
            while (spoolMotor.isBusy()) {
                telemetry.addData("pos: ", spoolMotor.getCurrentPosition());
                telemetry.update();
            }

            runToPosition(0);
            while (spoolMotor.isBusy()) {
                telemetry.addData("pos: ", spoolMotor.getCurrentPosition());
                telemetry.update();
            }



        }
        powerUpdater.interrupt();

    }

    private Thread powerUpdater = new Thread(() -> {
        while (!this.isStopRequested()) {
            if(EncoderTestingConfig.setPower != currentPower) {
                if(!spoolMotor.isBusy()) {
                    spoolMotor.setPower(EncoderTestingConfig.setPower);
                }
            }
        }
        spoolMotor.setPower(0);
    });

    private void runToPosition(int ticks) {
        if(spoolMotor.getPower() == 0) {
            spoolMotor.setPower(0.3);
        }
        spoolMotor.setTargetPosition(ticks);
        spoolMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
