package org.firstinspires.ftc.teamcode.Sandboxes.Michael.Unsafe;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

import static org.firstinspires.ftc.teamcode.Sandboxes.Dennis.PurePursuit.MovementConfig.CENTER_WHEEL_OFFSET;
import static org.firstinspires.ftc.teamcode.Sandboxes.Dennis.PurePursuit.MovementConfig.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Sandboxes.Dennis.PurePursuit.MovementConfig.TRACKWIDTH;
import static org.firstinspires.ftc.teamcode.Sandboxes.Dennis.PurePursuit.MovementConfig.WHEEL_DIAMETER;

@TeleOp(name="A -  Movement Testing", group="drive")
public class MovementTesting extends LinearOpMode {



    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private MotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private MotorEx encoderLeft, encoderRight, encoderPerp;

    private MecanumDrive robotDrive;

    private boolean pressingA = false;
    private boolean pressingB = false;

    private int ButtonACounter = 0;
    private int ButtonBCounter = 0;

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

        waitForStart();

        StartWaypoint p1 = new StartWaypoint(0, 0);
        GeneralWaypoint p2 = new GeneralWaypoint(0, 0, 0.8, 0.8, 30);
        EndWaypoint p3 = new EndWaypoint(400, 0, 0, 0.5, 0.5, 30, 0.8, 1);

        Path m_path = new Path(p1, p2, p3);
        m_path.init();

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
                m_path.followPath(robotDrive, holOdom);
                ButtonBCounter++;
                pressingB = false;
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