package org.firstinspires.ftc.teamcode.Sandboxes.Marc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.robotframework.controllers.DuckCarouselController;
@TeleOp
public class DuckTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DuckCarouselController ctrl = new DuckCarouselController(hardwareMap,telemetry);
        //DcMotor duckMotor = hardwareMap.get(DcMotor.class,"duck");
        waitForStart();
        boolean qA = false;
        while(opModeIsActive() && !isStopRequested())
        {
            if(gamepad1.a && !qA)
            {
                qA = true;
                telemetry.addData("init","yes i said init");
                telemetry.update();
              //  ctrl.initializeEnvironment();
                ctrl.spinOneDuck(true);
               /* DcMotor duckSpinner = hardwareMap.get(DcMotor.class, "duck");
                duckSpinner.setPower(true ? -.3 : .3);
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                duckSpinner.setPower(0);*/
                telemetry.addData("done","should be done");
                telemetry.update();

            }
            if(!gamepad1.a && qA)
            {
                qA = false;
            }
        }
    }
}
