package org.firstinspires.ftc.teamcode.Sandboxes.Michael.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.SlideController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.enums.IntakeSide;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideState;


@TeleOp(name="TeleOp Qualifier One", group="default")
public class TeleOpQual1 extends OpMode{
    private IntakeController BLUE_INTAKE;
    private IntakeController RED_INTAKE;
    private SlideController LINEAR_SLIDE;

    //private HashMap<Enum, GamepadKeys.Button> gamepadValues = new HashMap<>();

    private GamepadEx gamepad;
    private SampleMecanumDrive drive;
    private static FtcDashboard dashboard;


    volatile TeleOpState state = TeleOpState.STOPPED;
    public static Object stateMutex = new Object();





    @Override
    public void init() {
        BLUE_INTAKE = new IntakeController(hardwareMap, telemetry, IntakeSide.BLUE);
        RED_INTAKE = new IntakeController(hardwareMap, telemetry, IntakeSide.RED);
        LINEAR_SLIDE = new SlideController(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        gamepad = new GamepadEx(gamepad1);

        BLUE_INTAKE.setState(IntakeState.PARK);
        LINEAR_SLIDE.setState(SlideState.IN);

        setState(TeleOpState.IDLE);

        if(!writingHandler.isAlive()){
            writingHandler.start();
        }

    }

    @Override
    public void loop() {
        setState(TeleOpState.RUNNING);

        if(!roadrunnerHandlerRuntime.isAlive()){
            roadrunnerHandlerRuntime.start();
        }
        if(!buttonHandlerRuntime.isAlive()){
            buttonHandlerRuntime.start();
        }
    }

    @Override
    public void stop(){
        setState(TeleOpState.STOPPED);
        roadrunnerHandlerRuntime.interrupt();
        buttonHandlerRuntime.interrupt();
        writingHandler.interrupt();
    }

    public void setState(TeleOpState state){
        synchronized (stateMutex) {
            this.state = state;
        }
    }

    public Thread roadrunnerHandlerRuntime = new Thread(() -> {
        while(state.equals(TeleOpState.RUNNING)) {
            // Set Weighted Power
            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad.getLeftY(),
                            -gamepad.getLeftX(),
                            -gamepad.getRightX()
                    )
            );
            drive.update();
        }
    });

    public Thread buttonHandlerRuntime = new Thread(() -> {
        while(state.equals(TeleOpState.RUNNING)){
            BLUE_INTAKE.checkIntake();
           if(gamepad.isDown(GamepadKeys.Button.B)){
               if(gamepad.isDown(GamepadKeys.Button.DPAD_UP)){
                   LINEAR_SLIDE.setState(SlideState.TOP);
               }
               else if(gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)){LINEAR_SLIDE.setState(SlideState.BOTTOM);}
               else{LINEAR_SLIDE.setState(SlideState.MIDDLE);}
           }





       }
    });

    public Thread writingHandler = new Thread(() -> {
        while(!state.equals(TeleOpState.STOPPED)){
            Pose2d poseEstimate = drive.getPoseEstimate();
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            Canvas ftcField = telemetryPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, poseEstimate);

            //telemetryPacket.put("Estimated Pose X", poseEstimate.getX());
            //telemetryPacket.put("Estimated Pose Y", poseEstimate.getY());
            //telemetryPacket.put("Estimated Pose Heading", poseEstimate.getHeading());
            telemetryPacket.put("TeleOp State ", state);

            telemetry.addData("Slide state ", LINEAR_SLIDE.getState());

            telemetry.update();
            dashboard.sendTelemetryPacket(telemetryPacket);
        }
    });

}