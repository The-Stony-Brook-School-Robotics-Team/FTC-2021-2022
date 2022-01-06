package org.firstinspires.ftc.teamcode.Sandboxes.Michael.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop.enums.TeleOpRobotStates;
import org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.SlideController;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.sbs.bears.robotframework.controllers.IntakeController;
import org.sbs.bears.robotframework.enums.IntakeSide;
import org.sbs.bears.robotframework.enums.IntakeState;
import org.sbs.bears.robotframework.enums.SlideState;

import java.util.HashMap;


@TeleOp(name="TeleOp Qualifier One", group="default")
public class TeleOpQual1 extends OpMode{
    private IntakeController FRONT_INTAKE;
    private IntakeController BACK_INTAKE;
    private SlideController LINEAR_SLIDE;

    //private HashMap<Enum, GamepadKeys.Button> gamepadValues = new HashMap<>();

    private GamepadEx gamepad;
    private SampleMecanumDrive drive;
    private static FtcDashboard dashboard;


    volatile TeleOpState state = TeleOpState.STOPPED;
    public static Object stateMutex = new Object();

    double multiplier = 1;





    @Override
    public void init() {
        FRONT_INTAKE = new IntakeController(hardwareMap, telemetry, IntakeSide.FRONT);
        BACK_INTAKE = new IntakeController(hardwareMap, telemetry, IntakeSide.BACK);
        LINEAR_SLIDE = new SlideController(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        gamepad = new GamepadEx(gamepad1);

        FRONT_INTAKE.setState(IntakeState.PARK);
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
           FRONT_INTAKE.checkIntake();
           if(gamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                if(LINEAR_SLIDE.getState() != SlideState.IN){
                    LINEAR_SLIDE.setState(SlideState.IN);
                }
                else if(gamepad.isDown(GamepadKeys.Button.DPAD_UP)){
                    LINEAR_SLIDE.setState(SlideState.TOP);
                }
                else if(gamepad.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    LINEAR_SLIDE.setState(SlideState.BOTTOM);
                }
                else{LINEAR_SLIDE.setState(SlideState.MIDDLE);}

           }
           if(gamepad.wasJustPressed(GamepadKeys.Button.X)){
               if(multiplier == 1){multiplier = .3;}
               else{multiplier = 1;}
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
            dashboard.sendTelemetryPacket(telemetryPacket);
        }
    });

}