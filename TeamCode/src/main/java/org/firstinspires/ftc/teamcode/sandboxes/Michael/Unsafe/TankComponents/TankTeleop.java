package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe.TankComponents;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.sbs.bears.robotframework.enums.IntakeState;


@TeleOp(name = "A - TeleOp Tank")
public class TankTeleop extends OpMode {
    //Using Ramsete.
    SampleTankDrive drive;
    NewSlideController newSlideController;
    NewRedIntakeController newRedIntakeController;
    NewBlueIntakeController newBlueIntakeController;
    TapeController tapeController;
    boolean qLeft1 = false;
    boolean qRight1 = false;
    boolean qLeft2 = false;
    boolean qRight2 = false;
    boolean qUp2 = false;
    boolean qDown2 = false;
    boolean qB2 = false;
    boolean qX1 = false;
    boolean qY1 = false;
    boolean qUp1 = false;
    boolean qDown1 = false;

    private double tapeIncrement = .01;
    private double multiplier = 1;
    private boolean isClose = true;

    @Override
    public void init() {
        drive = new SampleTankDrive(hardwareMap);
        newSlideController = new NewSlideController(hardwareMap);
        newRedIntakeController = new NewRedIntakeController(hardwareMap, newSlideController.getClaw(), newSlideController.getDistanceSensor());
        newBlueIntakeController = new NewBlueIntakeController(hardwareMap, newSlideController.getClaw(), newSlideController.getDistanceSensor());
        tapeController = new TapeController(hardwareMap);
    }

    @Override
    public void start() {
        gamepad1Thread.start();
        gamepad2Thread.start();
        newRedIntakeController.setState(IntakeState.PARK);
        newBlueIntakeController.setState(IntakeState.PARK);
        tapeController.initServos();
        newSlideController.flipperOne.setPosition(SlideConstants.flipper_READY);
        newSlideController.flipperTwo.setPosition(SlideConstants.flipper_READY - SlideConstants.flipperOffset);

    }

    @Override
    public void loop() {


        drive.setMotorPowers(multiplier*(-gamepad1.left_stick_x + gamepad1.right_stick_x), multiplier*(-gamepad1.left_stick_x - gamepad1.right_stick_x));
        //drive.setMotorPowers(multiplier * -(gamepad1.left_stick_x - gamepad1.right_stick_x), multiplier * -(-gamepad1.left_stick_x - gamepad1.right_stick_x));
        newRedIntakeController.tick();
        newBlueIntakeController.tick();

    }
    @Override
    public void stop(){
        newSlideController.killThreads();
        gamepad1Thread.interrupt();
        gamepad2Thread.interrupt();
    }

    Thread gamepad1Thread = new Thread(){
        public void run(){
            while(!gamepad1Thread.isInterrupted()){
                if (gamepad1.b) {
                    if(!newSlideController.isExtendedPastThreshold()){
                        if(isClose){
                            newSlideController.extend(SlideConstants.slideMotorPosition_THREE_CLOSE, SlideConstants.flipper_THREE_CLOSE, SlideConstants.potentiometer_THREE_DEPOSIT);
                        }
                        else{newSlideController.extend(SlideConstants.slideMotorPosition_THREE_FAR, SlideConstants.flipper_THREE_FAR, SlideConstants.potentiometer_FAR);}

                    }
                    else{newSlideController.retract();}
                }
                if(gamepad1.a){
                    newSlideController.dropFreight();
                }
                if(gamepad1.dpad_left && !qLeft1) {
                    qLeft1 = true;
                    switch(newBlueIntakeController.getState()){
                        case DUMP:
                        case PARK:
                            newBlueIntakeController.setState(IntakeState.BASE);
                           break;
                        case BASE:
                            newBlueIntakeController.setState(IntakeState.DUMP);
                            break;
                    }
                }
                if(!gamepad1.dpad_left && qLeft1){
                    qLeft1 = false;
                }

                if(gamepad1.dpad_right && !qRight1){
                    qRight1 = true;
                    switch(newRedIntakeController.getState()){
                        case DUMP:
                        case PARK:
                            newRedIntakeController.setState(IntakeState.BASE);
                            break;
                        case BASE:
                            newRedIntakeController.setState(IntakeState.DUMP);
                            break;
                    }
                }
                if(!gamepad1.dpad_right && qRight1){
                    qRight1 = false;
                }
                if(gamepad1.x && !qX1){
                    qX1 = true;
                    if(multiplier==1) multiplier = 0.3;
                    else{multiplier = 1;}
                }
                if(!gamepad1.x && qX1){
                    qX1 = false;
                }
                if(gamepad1.y && !qY1){
                    qY1 = true;
                    isClose = !isClose;
                    if(isClose){
                        newSlideController.setTargetHeight(SlideConstants.potentiometer_THREE_DEPOSIT);
                    }
                    else{newSlideController.setTargetHeight(SlideConstants.potentiometer_FAR);}
                }
                if(!gamepad1.y && qY1){
                    qY1 = false;
                }
            }
        }
    };

    Thread gamepad2Thread = new Thread(){
        public void run(){
            while(!gamepad2Thread.isInterrupted()){
                if(gamepad2.b && !qB2){
                    qB2 = true;
                    tapeController.switchTape();
                }
                if(!gamepad2.b && qB2){
                    qB2 = false;
                }
                if(gamepad2.dpad_up && !qUp2){
                    qUp2 = true;
                    tapeController.tilt(tapeIncrement);
                }
                if(!gamepad2.dpad_up && qUp2){
                    qUp2 = false;
                }
                if(gamepad2.dpad_down && !qDown2){
                    qDown2 = true;
                    tapeController.tilt(-tapeIncrement);
                }
                if(!gamepad2.dpad_down && qDown2){
                    qDown2 = false;
                }
                if(gamepad2.dpad_left && !qLeft2){
                    qLeft2 = true;
                    tapeController.rotate(-tapeIncrement);
                }
                if(!gamepad2.dpad_left && qLeft2){
                    qLeft2 = false;
                }
                if(gamepad2.dpad_right && !qRight2){
                    qRight2 = true;
                    tapeController.rotate(tapeIncrement);
                }
                if(!gamepad2.dpad_right && qRight2){
                    qRight2 = false;
                }
                if(gamepad2.y) {
                    tapeController.extend(.7);
                }
                if(gamepad2.a){
                    tapeController.extend(.3);
                }
                if(!gamepad2.y && !gamepad2.a){
                    tapeController.extend(.5);
                }

            }
        }
    };





}