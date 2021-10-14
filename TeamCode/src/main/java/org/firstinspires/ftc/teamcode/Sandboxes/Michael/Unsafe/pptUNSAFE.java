package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe.util.customPath;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name="Michael Unsafe Testing", group="drive")
public class pptUNSAFE extends LinearOpMode {


    private static final double TRACKWIDTH = 12.75;
    private static final double CENTER_WHEEL_OFFSET = -8.7;
    private static final double WHEEL_DIAMETER = 2.0;
    private static final double TICKS_PER_REV = 8192;
    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx lf, rf, lb, rb;
    private OdometrySubsystem odometry;
    private MotorEx encoderLeft, encoderRight, encoderPerp;

    private MecanumDrive robotDrive;

    private boolean pressingA = false;
    private boolean pressingB = false;
    private boolean pressingY = false;

    private int ButtonACounter = 0;
    private int ButtonBCounter = 0;

    public static FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        lf = new MotorEx(hardwareMap, "lf");
        rf = new MotorEx(hardwareMap, "leftodom");
        lb = new MotorEx(hardwareMap, "backodom");
        rb = new MotorEx(hardwareMap, "rightodom");

        rf.setInverted(false);
        rb.setInverted(false);

        encoderLeft = new MotorEx(hardwareMap, "leftodom");
        encoderRight = new MotorEx(hardwareMap, "rightodom");
        encoderPerp = new MotorEx(hardwareMap, "backodom");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        encoderLeft.resetEncoder();
        encoderRight.resetEncoder();
        encoderPerp.resetEncoder();


        robotDrive = new MecanumDrive(lf, rf, lb, rb);
        dashboard = FtcDashboard.getInstance();

        HolonomicOdometry holOdom = new HolonomicOdometry(
                () -> encoderLeft.getCurrentPosition() * TICKS_TO_INCHES,
                () -> -(encoderRight.getCurrentPosition() * TICKS_TO_INCHES),
                () -> (encoderPerp.getCurrentPosition() * TICKS_TO_INCHES),
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        odometry = new OdometrySubsystem(holOdom);

        //com.arcrobotics.ftclib.geometry.Pose2d currentPose = new com.arcrobotics.ftclib.geometry.Pose2d(odometry.getPose().getX(), odometry.getPose().getY(), odometry.getPose().getRotation());
        Waypoint p1 = new StartWaypoint(0.0, 0.0);
        Waypoint p2 = new GeneralWaypoint(
                10, 10,
                .3,
                1,
                5
        );
        Waypoint p3 = new GeneralWaypoint(
                20, 20,
                -odometry.getPose().getY() + 20,
                .8,
                1,
                5
        );
        //com.arcrobotics.ftclib.geometry.Pose2d endPose = new com.arcrobotics.ftclib.geometry.Pose2d(currentPose.getX() + 10, currentPose.getY(), currentPose.getRotation());
        Waypoint p4 = new EndWaypoint(
                30, 30, 0,
                .8,
                1,
                5,
                0.4,
                0.4
        );
        Waypoint p5 = new EndWaypoint(
                odometry.getPose().getX() + 10,
                odometry.getPose().getY(),
                0, .8, .3, 5, .4, .4
        );

        customPath path = new customPath(p1, p2, p3, p4);
        customPath newpath = new customPath(p1, p5);
        Path vanillaPath = new Path(p1, p2, p3, p4);

        waitForStart();


        while (opModeIsActive() && !isStopRequested())
        {
            odometry.update();

            if(gamepad1.a && !pressingA) {
                pressingA = true;
            } else if(!gamepad1.a && pressingA) {
                encoderLeft.resetEncoder();
                encoderRight.resetEncoder();
                encoderPerp.resetEncoder();
                ButtonACounter++;
                pressingA = false;
            }
            if(gamepad1.b && !pressingB) {
                pressingB = true;
            } else if(!gamepad1.b && pressingB) {
                newpath.followPath(robotDrive, odometry);
            }
            if(gamepad1.y && !pressingY) {
                pressingY = true;
            } else if(!gamepad1.y && pressingY) {
                vanillaPath.followPath(robotDrive, holOdom);
            }


            lf.set(0.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x));
            rf.set(0.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x));
            lb.set(0.6 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x));
            rb.set(0.6 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x));


            TelemetryPacket telemPacket = new TelemetryPacket();
            Canvas ftcField = telemPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, new Pose2d(odometry.getPose().getX(), -(odometry.getPose().getY()), -(odometry.getPose().getHeading())));

            telemPacket.put("Robot Test", 1);
            telemPacket.put("Estimated Pose X", odometry.getPose().getX());
            telemPacket.put("Estimated Pose Y", odometry.getPose().getY());
            telemPacket.put("Estimated Pose Heading", Math.toDegrees(odometry.getPose().getHeading()));

            dashboard.sendTelemetryPacket(telemPacket);

            telemetry.addData("A Counter", ButtonACounter);
            telemetry.addData("B Counter", ButtonBCounter);
            telemetry.addData("Left Encoder Position", encoderLeft.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.addData("Right Encoder Position", encoderRight.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.addData("Back Encoder Position", encoderPerp.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.update();


        }



    }


}