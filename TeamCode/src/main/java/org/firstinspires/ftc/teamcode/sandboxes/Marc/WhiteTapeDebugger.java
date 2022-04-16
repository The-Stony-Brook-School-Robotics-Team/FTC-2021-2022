package org.firstinspires.ftc.teamcode.sandboxes.Marc;

import com.qualcomm.hardware.broadcom.BroadcomColorSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class WhiteTapeDebugger extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 colorNew;
        colorNew = hardwareMap.get(RevColorSensorV3.class, "color");
        colorNew.write8(BroadcomColorSensor.Register.LS_MEAS_RATE,0x01010000); // see pdf page 20 // increase speed
        colorNew.write8(BroadcomColorSensor.Register.PS_MEAS_RATE,0x00000001); // see pdf page 19 // increase speed
        //colorNew.setGain(Configuration.colorSensorGain);


        waitForStart();

        while(opModeIsActive() && !isStopRequested())
        {
           telemetry.addData("alpha",colorNew.getNormalizedColors().alpha);
           telemetry.addData("r",colorNew.getNormalizedColors().red);
           telemetry.addData("g",colorNew.getNormalizedColors().green);
           telemetry.addData("b",colorNew.getNormalizedColors().blue);
           telemetry.addData("isOnWhiteTape",colorNew.getNormalizedColors().alpha > 0.99985);
           telemetry.update();
        }
    }

}


