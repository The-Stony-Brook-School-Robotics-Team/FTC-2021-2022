package org.firstinspires.ftc.teamcode.Sandboxes.Dennis.teleop;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import java.util.HashMap;

public class TeleOpRoutine extends OpMode {

    /**
     * Teleop Routine States
     */
    private enum TELEOP_ROUTINE_STATES {
        STOPPED(0), INITIALIZING(1), INITIALIZED(2), RUNNING(3);
        private int ident;
        TELEOP_ROUTINE_STATES(int ident) {
            this.ident = ident;
        }
    }
    private TELEOP_ROUTINE_STATES ROBOT_STATE = TELEOP_ROUTINE_STATES.STOPPED;

    /**
     * Item Ports
     */
    private HashMap<String, DcMotorEx> MOTOR_PORTS = new HashMap<>();
    private HashMap<String, Servo> SERVO_PORTS = new HashMap<>();

    /**
     * LED Driver
     */
    private static RevBlinkinLedDriver blinkinLedDriver;

    /**
     * Roadrunner Things
     */
    private static SampleMecanumDrive drive;
    private static FtcDashboard dashboard;

    @Override
    public void init() {
        ROBOT_STATE = TELEOP_ROUTINE_STATES.STOPPED;
        telemetry.addLine("Robot Ready");
    }

    @Override
    public void loop() {
        switch(ROBOT_STATE) {
            case STOPPED:
                initialize();
                LIGHT_HANDLER_ROUTINE.start();
                ROADRUNNER_HANDLER_ROUTINE.start();

        }
    }

    public void initialize() {
        // LED DRIVER
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "colorstrip");
        // MOTORS
        MOTOR_PORTS.put("lf", hardwareMap.get(DcMotorEx.class, "lf"));
        MOTOR_PORTS.put("rf", hardwareMap.get(DcMotorEx.class, "rf"));
        MOTOR_PORTS.put("lb", hardwareMap.get(DcMotorEx.class, "lb"));
        MOTOR_PORTS.put("rf", hardwareMap.get(DcMotorEx.class, "rf"));
        // ODOM
        MOTOR_PORTS.put("left", hardwareMap.get(DcMotorEx.class, "left"));
        MOTOR_PORTS.put("right", hardwareMap.get(DcMotorEx.class, "right"));
        MOTOR_PORTS.put("center", hardwareMap.get(DcMotorEx.class, "center"));
        // ROADRUNNER
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dashboard = FtcDashboard.getInstance();
    }


    /**
     * Roadrunner Handler
     */
    public Thread ROADRUNNER_HANDLER_ROUTINE = new Thread(() -> {
        // Set Weighted Power
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        drive.update();
        Pose2d poseEstimate = drive.getPoseEstimate();
        // Teleme Init
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        Canvas ftcField = telemetryPacket.fieldOverlay();
        DashboardUtil.drawRobot(ftcField, poseEstimate);
        // Telemetry Packet Update
        telemetryPacket.put("Estimated Pose X", poseEstimate.getX());
        telemetryPacket.put("Estimated Pose Y", poseEstimate.getY());
        telemetryPacket.put("Estimated Pose Heading", poseEstimate.getHeading());
    });

    public Thread LIGHT_HANDLER_ROUTINE = new Thread(() -> {
        while(!Thread.interrupted()) {
            switch (ROBOT_STATE) {
                case STOPPED:
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED);
                    break;

                case INITIALIZING:
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    break;

                case INITIALIZED:
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

                case RUNNING:
                    // TODO: HANDLE COLOR OPERATIONS
            }
        }
    });
}

