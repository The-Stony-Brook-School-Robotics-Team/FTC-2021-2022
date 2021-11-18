package org.firstinspires.ftc.teamcode.sandboxes.Dennis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(group = "default", name = "U - Three Wheel Tracking Test")
public class TrackingTest extends LinearOpMode {
    private static final double TRACKWIDTH = 12.75;
    private static final double CENTER_WHEEL_OFFSET = -8.7;

    private static final double WHEEL_DIAMETER = 2.0;
    private static final double TICKS_PER_REV = 8192;
    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    private static HolonomicOdometry holOdom;
    private static OdometrySubsystem odometry;
    private static FtcDashboard dashboard;

    private MotorEx leftodom, rightodom, centerodom;
    private DcMotorEx lf, rf, rb, lb;

    @Override
    public void runOpMode() throws InterruptedException {
        // Init
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftodom = new MotorEx(hardwareMap, "leftodom");
        rightodom = new MotorEx(hardwareMap, "rightodom");
        centerodom = new MotorEx(hardwareMap, "centerodom");

        leftodom.setDistancePerPulse(TICKS_TO_INCHES);
        rightodom.setDistancePerPulse(TICKS_TO_INCHES);
        centerodom.setDistancePerPulse(TICKS_TO_INCHES);

        leftodom.resetEncoder();
        rightodom.resetEncoder();
        centerodom.resetEncoder();

        holOdom = new HolonomicOdometry(
                () -> (leftodom.getCurrentPosition() * TICKS_TO_INCHES),
                () -> -(rightodom.getCurrentPosition() * TICKS_TO_INCHES),
                () -> (centerodom.getCurrentPosition() * TICKS_TO_INCHES),
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );


        dashboard = FtcDashboard.getInstance();
        odometry = new OdometrySubsystem(holOdom);

        waitForStart();

        while(!isStopRequested()) {
            odometry.update();


            // Weighted Driving
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);
            // End


            TelemetryPacket telemPacket = new TelemetryPacket();
            Canvas ftcField = telemPacket.fieldOverlay();
            DashboardUtil.drawRobot(ftcField, new Pose2d(odometry.getPose().getX(), -(odometry.getPose().getY()), -(odometry.getPose().getHeading())));

            telemPacket.put("Robot Test", 1);
            telemPacket.put("Estimated Pose X", odometry.getPose().getX());
            telemPacket.put("Estimated Pose Y", odometry.getPose().getY());
            telemPacket.put("Estimated Pose Heading", Math.toDegrees(odometry.getPose().getHeading()));

            dashboard.sendTelemetryPacket(telemPacket);
        }


    }
}
