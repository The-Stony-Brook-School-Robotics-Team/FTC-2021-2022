package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.enums.LiftStates;

@TeleOp(name="Controller Tester", group="Linear Opmode")
public class ControllerTester extends LinearOpMode {
    private IntakeController intake;

    public void runOpMode() throws InterruptedException {
        intake = new IntakeController(hardwareMap, telemetry);
        intake.setState(LiftStates.BASE);
        waitForStart();

        while(opModeIsActive()){

            intake.checkIntake();

            if(gamepad1.a){intake.setState(LiftStates.BASE);}


            telemetry.addData("State: ", intake.getState());
            telemetry.addData("is ", intake.isObjectInPayload());
            telemetry.update();
        }


    }
}
