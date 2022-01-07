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

    private GamepadEx gamepad;
    private SampleMecanumDrive drive;
    private static FtcDashboard dashboard;

    volatile TeleOpState state = TeleOpState.STOPPED;
    public static Object stateMutex = new Object();

    @Override
    public void init() {
        /** Two Intake objects, one for each side. Linear Slide controller subject to change, TBD */

        BLUE_INTAKE = new IntakeController(hardwareMap, telemetry, IntakeSide.BLUE);
        RED_INTAKE = new IntakeController(hardwareMap, telemetry, IntakeSide.RED);
        LINEAR_SLIDE = new SlideController(hardwareMap, telemetry);


        drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        /** Wrapper of the gamepad class */
        gamepad = new GamepadEx(gamepad1);

        /** Starting states */
        BLUE_INTAKE.setState(IntakeState.PARK);
        LINEAR_SLIDE.setState(SlideState.IN);

        /** Set this Teleop state after initializing the hardware in order to avoid null references */
        setState(TeleOpState.IDLE);

        /** Start the threads that should be running once robot is initialized */
        if(!writingHandler.isAlive()){
            writingHandler.start();
        }

    }

    @Override
    public void loop() {
        setState(TeleOpState.RUNNING);

        /** Start the threads that should be running once robot is running */
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

        /** Stop all the threads */
        roadrunnerHandlerRuntime.interrupt();
        buttonHandlerRuntime.interrupt();
        writingHandler.interrupt();
    }

    /** Sets the robot to the desired state, synchronized to lock to one thread
     * @param state The desired state to set the robot to. */
    public void setState(TeleOpState state){
        synchronized (stateMutex) {
            this.state = state;
        }
    }

    /** Thread that handles driving via Roadrunner weighted drive power */
    public Thread roadrunnerHandlerRuntime = new Thread(() -> {
        while(state.equals(TeleOpState.RUNNING)) {
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

    /** Thread that handles all actions bound to user input. */
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

    /** Thread that handles all writing of information to both the dashboard and telemetry. */
    public Thread writingHandler = new Thread(() -> {
        while(!state.equals(TeleOpState.STOPPED)){
            Pose2d poseEstimate = drive.getPoseEstimate();
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            Canvas ftcField = telemetryPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, poseEstimate);

            telemetryPacket.put("TeleOp State ", state);

            dashboard.sendTelemetryPacket(telemetryPacket);
        }
    });

}