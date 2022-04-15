package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.sbs.bears.robotframework.enums.IntakeState;


@TeleOp(name = "tank drive", group = "drive")
public class TankTeleop extends OpMode {
    //Using Ramsete.
    SampleTankDrive drive;
    NewSlideController newSlideController;
    NewIntakeController newIntakeController;
    boolean pressingA = false;
    boolean pressingB = false;
    @Override
    public void init() {
        drive = new SampleTankDrive(hardwareMap);
        newSlideController = new NewSlideController(hardwareMap);
        newIntakeController = new NewIntakeController(hardwareMap, newSlideController.getClaw(), newSlideController.getDistanceSensor());
    }

    @Override
    public void start() {
        gamepadThread.start();
    }

    @Override
    public void loop() {
        drive.setMotorPowers(-gamepad1.left_stick_x +gamepad1.right_stick_x, -gamepad1.left_stick_x - gamepad1.right_stick_x);
        //drive.setMotorPowers(-gamepad1.left_stick_y +gamepad1.right_stick_x, -gamepad1.left_stick_y - gamepad1.right_stick_x);
        drive.update();
        newIntakeController.tick();
        telemetry.addData("distance", newIntakeController.intakeSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("state", newIntakeController.getState());
        telemetry.update();

    }
    @Override
    public void stop(){
        newSlideController.killThreads();
        gamepadThread.interrupt();
    }

    Thread gamepadThread = new Thread(){
        public void run(){
            while(!gamepadThread.isInterrupted()){
                if (gamepad1.b) {
                    if(!newSlideController.isExtendedPastThreshold()){
                        newSlideController.extend(SlideConstants.slideMotorPosition_THREE_DEPOSIT);
                    }
                    else{newSlideController.retract();}
                }
                if(gamepad1.a){
                    newSlideController.dropFreight();
                }
                if(gamepad1.dpad_left){
                    newIntakeController.setState(IntakeState.BASE);
                }
                if(gamepad1.dpad_right){
                    newIntakeController.setState(IntakeState.DUMP);
                }
            }
        }
    };


}