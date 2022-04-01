package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.PointTurnWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name="awful", group="drive")
//@Disabled
@Deprecated
public class PurePursuitTesting extends LinearOpMode {

    //TODO MOST LIKELY BROKEN BE CAREFUL LMAOO
    private static final double TRACKWIDTH = 9.5;
    private static final double CENTER_WHEEL_OFFSET = -.62;
    private static final double WHEEL_DIAMETER = .738188976*2;
    private static final double TICKS_PER_REV = 8192;
    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;



    private Motor lf, rf, lb, rb;
    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private MotorEx encoderLeft, encoderRight, encoderPerp;


    private boolean pressingA = false;


    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = new Motor(hardwareMap, "lf");
        rf = new Motor(hardwareMap, "rf");
        lb = new Motor(hardwareMap, "lb");
        rb = new Motor(hardwareMap, "rb");


        lf.setInverted(true);
        rf.setInverted(true);

        encoderLeft = new MotorEx(hardwareMap, "leftodom");
        encoderRight = new MotorEx(hardwareMap, "rightodom");
        encoderPerp = new MotorEx(hardwareMap, "duck");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        encoderLeft.resetEncoder();
        encoderRight.resetEncoder();
        encoderPerp.resetEncoder();



        dashboard = FtcDashboard.getInstance();

        HolonomicOdometry holOdom = new HolonomicOdometry(
                () -> encoderRight.getCurrentPosition() * TICKS_TO_INCHES,
                () -> -(encoderLeft.getCurrentPosition() * TICKS_TO_INCHES),
                () -> -(encoderPerp.getCurrentPosition() * TICKS_TO_INCHES),
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );
        drive = new MecanumDrive(false, lf, rf, lb, rb);
        odometry = new OdometrySubsystem(holOdom);


        Waypoint p1 = new StartWaypoint(odometry.getPose());
        Waypoint p2 = new GeneralWaypoint(new com.arcrobotics.ftclib.geometry.Pose2d(10,10,  odometry.getPose().getRotation()), 10, 10, 1);
        Waypoint p3 = new EndWaypoint(
                15, 15, Math.PI/2, 10,
                10, 1,
                .5, .5
        );
        Path path = new Path(p1, p2, p3);
        path.init();
        waitForStart();


        while (opModeIsActive() && !isStopRequested())
        {
            path.followPath(drive, holOdom);
            odometry.update();
            holOdom.updatePose();






            TelemetryPacket telemPacket = new TelemetryPacket();
            Canvas ftcField = telemPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, new Pose2d(odometry.getPose().getX(), (odometry.getPose().getY()), (odometry.getPose().getHeading())));

            telemPacket.put("Estimated Pose X", odometry.getPose().getX());
            telemPacket.put("Estimated Pose Y", odometry.getPose().getY());
            telemPacket.put("Estimated Pose Heading", Math.toDegrees(odometry.getPose().getHeading()));

            dashboard.sendTelemetryPacket(telemPacket);


            telemetry.addData("Left Encoder Position", encoderLeft.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.addData("Right Encoder Position", encoderRight.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.addData("Back Encoder Position", encoderPerp.getCurrentPosition() * TICKS_TO_INCHES);
            telemetry.update();
        }


    }


}