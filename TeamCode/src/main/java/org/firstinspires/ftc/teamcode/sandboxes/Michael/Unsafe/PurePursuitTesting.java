package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

@TeleOp(name="awful", group="drive")
//@Disabled
//@Deprecated
public class PurePursuitTesting extends LinearOpMode {


    private static final double TRACKWIDTH = 9.5;
    private static final double CENTER_WHEEL_OFFSET = -.62;
    private static final double WHEEL_DIAMETER = .738188976*2;
    private static final double TICKS_PER_REV = 8192;
    private static final double TICKS_TO_INCHES = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;



    private DcMotorEx lf, rf, lb, rb;
    private MecanumDrive drive;
    private OdometrySubsystem odometry;
    private MotorEx encoderLeft, encoderRight, encoderPerp;


    private boolean pressingA = false;


    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);

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
        odometry = new OdometrySubsystem(holOdom);

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
                pressingA = false;
            }


            // Weighted Driving
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
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