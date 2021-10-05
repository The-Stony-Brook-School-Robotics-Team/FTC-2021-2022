package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Final;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name="PurePursuit Testing Safe", group="drive")
public class PurePursuitTestingSafe extends LinearOpMode {


    private static final double TRACKWIDTH = 12.75;
    private static final double CENTER_WHEEL_OFFSET = -8.7;
    private static final double WHEEL_DIAMETER = 2.0;
    private static final double TICKS_PER_REV = 8192;
    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private MotorEx encoderLeft, encoderRight, encoderPerp;

    private PurePursuitCommand ppCommand;
    private MecanumDrive robotDrive;

    boolean pressingA = false;
    boolean pressingB = false;

    FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        lf = new MotorEx(hardwareMap, "lf");
        rf = new MotorEx(hardwareMap, "leftodom");
        lb = new MotorEx(hardwareMap, "backodom");
        rb = new MotorEx(hardwareMap, "rightodom");

        encoderLeft = new MotorEx(hardwareMap, "leftodom");
        encoderRight = new MotorEx(hardwareMap, "rightodom");
        encoderPerp = new MotorEx(hardwareMap, "backodom");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        robotDrive = new MecanumDrive(lf, rf, lb, rb);
        dashboard = FtcDashboard.getInstance();
        
        HolonomicOdometry holOdom = new HolonomicOdometry(
                () -> encoderLeft.getCurrentPosition() * TICKS_TO_INCHES,
                () -> -(encoderRight.getCurrentPosition() * TICKS_TO_INCHES),
                () -> (encoderPerp.getCurrentPosition() * TICKS_TO_INCHES),
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry = new OdometrySubsystem(holOdom);

        waitForStart();

        ppCommand = new PurePursuitCommand(
                robotDrive, odometry,
                new StartWaypoint(0,0),
                new GeneralWaypoint(200,0,0.8,0.8,30),
                new EndWaypoint(
                        400, 0, 0, 0.5,
                        0.5, 30, 0.8, 1
                )
        );

        while (opModeIsActive() && !isStopRequested())
        {
            odometry.update();

            if(gamepad1.a != pressingA) {
                pressingA = true;
            } else if(!gamepad1.a && pressingA) {
                encoderLeft.set(0);
                encoderRight.set(0);
                encoderPerp.set(0);
                pressingA = false;
            }

            if(gamepad1.b != pressingB) {
                pressingB = true;
            } else if(!gamepad1.b && pressingB) {
                ppCommand.schedule();
                pressingB = false;
            }

            TelemetryPacket telemPacket = new TelemetryPacket();
            Canvas ftcField = telemPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, new Pose2d(odometry.getPose().getX(), odometry.getPose().getY(), -(odometry.getPose().getHeading())));
            
            telemPacket.put("Estimated Pose X", odometry.getPose().getX());
            telemPacket.put("Estimated Pose Y", odometry.getPose().getY());
            
            dashboard.sendTelemetryPacket(telemPacket);

            telemetry.addData("Left Encoder Position", encoderLeft.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.addData("Right Encoder Position", encoderRight.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.addData("Back Encoder Position", encoderPerp.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.update();
        }


    }


}