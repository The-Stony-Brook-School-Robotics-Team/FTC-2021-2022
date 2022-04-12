package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.sbs.bears.robotframework.controllers.IntakeControllerBlue;
import org.sbs.bears.robotframework.controllers.IntakeControllerRed;
import org.sbs.bears.robotframework.controllers.SlideController;
import org.sbs.bears.robotframework.enums.IntakeState;

@TeleOp(name = "Intake Tester")
public class IntakeTester extends OpMode {
    SampleTankDrive drive;
    IntakeControllerBlue intakeControllerBlue;
    IntakeControllerRed intakeControllerRed;
    SlideController slideController;
    boolean pressingA = false;
    boolean pressingB = false;
    @Override
    public void init() {
        drive = new SampleTankDrive(hardwareMap);
        slideController = new SlideController(hardwareMap, telemetry);
        intakeControllerBlue = new IntakeControllerBlue(hardwareMap, slideController.blueDumperServo, telemetry);
        intakeControllerRed = new IntakeControllerRed(hardwareMap, slideController.redDumperServo, telemetry);
        intakeControllerBlue.setState(IntakeState.DUMP);
        slideController.initTeleop();
    }

    @Override
    public void loop() {
        drive.setMotorPowers(-gamepad1.left_stick_y + gamepad1.right_stick_x, -gamepad1.left_stick_y - gamepad1.right_stick_x);

        if(gamepad1.dpad_left){
            intakeControllerBlue.setState(IntakeState.BASE);
        }
        if(intakeControllerBlue.getState() == IntakeState.BASE){
            intakeControllerBlue.checkIntake();
        }
        if(gamepad1.dpad_right){
            intakeControllerBlue.setState(IntakeState.DUMP);
        }
        if(gamepad1.a){
            slideController.extendSlide();
        }
        if(gamepad1.b){
            slideController.retractSlide();
        }
        if(gamepad1.x){
            slideController.dropCube();
        }
        drive.update();
        telemetry.addData("distance", intakeControllerBlue.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.update();
    }
}


