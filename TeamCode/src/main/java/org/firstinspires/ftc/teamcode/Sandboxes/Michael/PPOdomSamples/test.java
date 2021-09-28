package org.firstinspires.ftc.teamcode.Sandboxes.Michael.PPOdomSamples;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="emTesting", group="drive")
public class test extends OpMode {

    private static Pose2d robotPose;
    // imu and hwMp
    private static BNO055IMU imu;
    // Motors
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    // Odom
    private HolonomicOdometry holOdom;
    private MotorEx lOdom;
    private MotorEx rOdom;
    private MotorEx bOdom;
    // Odom Offsets
    public static double WHEEL_RADIUS = 1.97;
    public static double GEAR_RATIO = 8.0 / 14;
    public static double TRACK_WIDTH = 12.75;
    public static final double TICKS_PER_REV = 145.6;
    public static double WHEEL_DIAMETER = WHEEL_RADIUS * 2;
    public static final double TICKS_TO_INCHES  = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;
    public static final double CENTER_WHEEL_OFFSET = -8.7;
    // Encoders
    private DcMotor lfencoder;
    private DcMotor rfencoder;
    private DcMotor lbencoder;
    private DcMotor rbencoder;
    @Override
    public void init() {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "leftodom");
        lb = hardwareMap.get(DcMotor.class, "backodom");
        rb = hardwareMap.get(DcMotor.class, "rightodom");

        lOdom = new MotorEx(hardwareMap, "leftodom");
        rOdom = new MotorEx(hardwareMap, "rightodom");
        bOdom = new MotorEx(hardwareMap, "backodom");

        lOdom.setDistancePerPulse(TICKS_TO_INCHES);
        rOdom.setDistancePerPulse(TICKS_TO_INCHES);
        bOdom.setDistancePerPulse(TICKS_TO_INCHES);

        lfencoder = hardwareMap.get(DcMotor.class, "lfencoder");
        rfencoder = hardwareMap.get(DcMotor.class, "rfencoder");
        lbencoder = hardwareMap.get(DcMotor.class, "lbencoder");
        rbencoder = hardwareMap.get(DcMotor.class, "rbencoder");

        holOdom = new HolonomicOdometry(
                lOdom::getDistance,
                rOdom::getDistance,
                bOdom::getDistance,
                TRACK_WIDTH,
                CENTER_WHEEL_OFFSET
        );

        lfencoder.setDirection(DcMotorSimple.Direction.REVERSE);
        lbencoder.setDirection(DcMotorSimple.Direction.REVERSE);

        lfencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        holOdom.updatePose();
        robotPose = holOdom.getPose();
        telemetry.addData("im here", robotPose);
    }

    @Override
    public void stop() {

    }

}