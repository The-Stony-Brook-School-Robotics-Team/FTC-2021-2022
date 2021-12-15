package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.autonomous.AutonomousMode;
import org.sbs.bears.robotframework.Robot;
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
