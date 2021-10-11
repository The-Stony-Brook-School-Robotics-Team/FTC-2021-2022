package org.firstinspires.ftc.teamcode.Sandboxes.William;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BearsUtil.MotorEncoderController;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;

@TeleOp(name = "PP Test by Will", group = "PP")
public class PP extends OpMode {
    public static final double WHEEL_DIAMETER = 2.0;
    public static final double TICKS_PER_REV = 8192;
    public static final double TICKS_TO_INCHES = TICKS_PER_REV * WHEEL_DIAMETER * Math.PI;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    public static final double TRACK_WIDTH = 12.75;
    public static final double CENTER_WHEEL_OFFSET = -8.7;
    private static final double MINIMUM_STOP_DISTANCE = 10;  //inches
    private static final double ACCEPTABLE_ERROR = 0.5;    //inches
    MotorEncoderController motorCtrls;
    FtcDashboard dashboard;
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private double xRealTimeValue;
    private double yRealTimeValue;
    private double xTargetValue = 24;    //inches
    private double yTargetValue = 24;    //inches
    private double xDistanceToTarget;   //inches; can be negative
    private double yDistanceToTarget;   //inches; can be negative
    private double totalDistance;       //inches; always positive


    private OdometrySubsystem odometry;
    private MotorEx encoderLeft, encoderRight, encoderPerp;

    private MecanumDrive robotDrive;

    public void PositionTracker() {
        MotorEx leftFront, rightFront, leftBack, rightBack;
        leftFront = new MotorEx(hardwareMap, "lf");
        rightFront = new MotorEx(hardwareMap, "leftodom");
        leftBack = new MotorEx(hardwareMap, "backodom");
        rightBack = new MotorEx(hardwareMap, "rightodom");

        encoderLeft = new MotorEx(hardwareMap, "leftodom");
        encoderRight = new MotorEx(hardwareMap, "rightodom");
        encoderPerp = new MotorEx(hardwareMap, "backodom");

        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        encoderLeft.resetEncoder();
        encoderRight.resetEncoder();
        encoderPerp.resetEncoder();

        robotDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);
        dashboard = FtcDashboard.getInstance();

        HolonomicOdometry holOdom = new HolonomicOdometry(
                () -> encoderLeft.getCurrentPosition() * TICKS_TO_INCHES,
                () -> -(encoderRight.getCurrentPosition() * TICKS_TO_INCHES),
                () -> (encoderPerp.getCurrentPosition() * TICKS_TO_INCHES),
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );
        odometry = new OdometrySubsystem(holOdom);
    }

    public void UpdatePositions() {
        PositionTracker();
        xRealTimeValue = odometry.getPose().getX();
        yRealTimeValue = odometry.getPose().getY();
        xDistanceToTarget = xTargetValue - xRealTimeValue;
        yDistanceToTarget = yTargetValue - yRealTimeValue;
        totalDistance = GetTotalDistance();
    }

    @Override
    public void init() {
        motorCtrls = new MotorEncoderController(hardwareMap, telemetry);
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            double xLeftStickValue;     //right positive; left negative
            double yLeftStickValue;     //forward positive; back negative
            UpdatePositions();
            xLeftStickValue = GetXStickValue();
            yLeftStickValue = GetYStickValue();
            lf.setPower(0.5 * (yLeftStickValue + xLeftStickValue /*+ gamepad1.right_stick_x*/));
            rf.setPower(0.5 * (yLeftStickValue - xLeftStickValue /*- gamepad1.right_stick_x*/));
            lb.setPower(0.5 * (yLeftStickValue - xLeftStickValue /*+ gamepad1.right_stick_x*/));
            rb.setPower(0.5 * (yLeftStickValue + xLeftStickValue /*- gamepad1.right_stick_x*/));
            totalDistance = GetTotalDistance();
            telemetry.addData("Distance to destination", totalDistance);
            if (totalDistance < ACCEPTABLE_ERROR)
                while (true) ;
            while(gamepad1.x);
        }
    }

    public double GetXStickValue() {
        if (xDistanceToTarget > MINIMUM_STOP_DISTANCE)
            return -xDistanceToTarget / totalDistance;   //-sin(theta)
        else if (xRealTimeValue < ACCEPTABLE_ERROR)
            return xRealTimeValue / MINIMUM_STOP_DISTANCE;
        else
            return 0;
    }

    public double GetYStickValue() {
        if (yDistanceToTarget > MINIMUM_STOP_DISTANCE)
            return yDistanceToTarget / totalDistance;    //cos(theta)
        else if (yRealTimeValue < ACCEPTABLE_ERROR)
            return yRealTimeValue / MINIMUM_STOP_DISTANCE;
        else
            return 0;
    }

    public double GetTotalDistance() {
        return Math.sqrt(Math.pow(xDistanceToTarget, 2) + Math.pow(yDistanceToTarget, 2));
    }
}
