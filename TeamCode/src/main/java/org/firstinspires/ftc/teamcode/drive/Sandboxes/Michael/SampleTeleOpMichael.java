package org.firstinspires.ftc.teamcode.drive.Sandboxes.Michael;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.drive.BearsUtil.MotorEncoderController;


@TeleOp(name="shoot me and make me die", group="drive")
public class SampleTeleOpMichael extends OpMode {



    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private DcMotor lOdom;
    private DcMotor rOdom;
    private DcMotor bOdom;
    private DcMotor lfencoder;
    private DcMotor rfencoder;
    private DcMotor lbencoder;
    private DcMotor rbencoder;



    private HolonomicOdometry odometry;

    public static double WHEEL_RADIUS = 1.97;
    public static double WHEEL_DIAMETER = 3.94;
    public static double GEAR_RATIO = 8.0 / 14;
    public static double TRACK_WIDTH = 12.75;
    public static final double TICKS_PER_REV = 145.6;
    public static final double CENTER_WHEEL_OFFSET = -8.7;

    //public static final double TRACKWIDTH = ;
    //public static final double CENTER_WHEEL_OFFSET = 0.477;
    //public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    //public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    static final double TICKS_TO_INCHES = 15.3;
    BNO055IMU imu;
    public static Pose2d robotPose;



    @Override
    public void init() {
        //lf = hardwareMap.get(DcMotor.class, "lf");
        //rf = hardwareMap.get(DcMotor.class, "leftodom");
        //lb = hardwareMap.get(DcMotor.class, "backodom");
        //rb = hardwareMap.get(DcMotor.class, "rightodom");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        Motor lf = new Motor(hardwareMap, "lf", Motor.GoBILDA.RPM_1150);
        Motor rf = new Motor(hardwareMap, "leftodom", Motor.GoBILDA.RPM_1150);
        Motor lb = new Motor(hardwareMap, "backodom", Motor.GoBILDA.RPM_1150);
        Motor rb = new Motor(hardwareMap, "rightodom", Motor.GoBILDA.RPM_1150);



        //private Motor.Encoder leftOdom = new Motor.Encoder(hardwareMap, "leftodom", );

        MecanumDrive mecanum = new MecanumDrive(lf, rf,
                lb, rb);

        //lOdom = new MotorEx(hardwareMap, "leftodom");
        //rOdom = new MotorEx(hardwareMap, "rightodom");
        //bOdom = new MotorEx(hardwareMap, "backodom");



        //lOdom.setDistancePerPulse(DISTANCE_PER_PULSE);
        //rOdom.setDistancePerPulse(DISTANCE_PER_PULSE);
        //bOdom.setDistancePerPulse(DISTANCE_PER_PULSE);


        lfencoder = hardwareMap.get(DcMotor.class, "lfencoder");
        rfencoder = hardwareMap.get(DcMotor.class, "rfencoder");
        lbencoder = hardwareMap.get(DcMotor.class, "lbencoder");
        rbencoder = hardwareMap.get(DcMotor.class, "rbencoder");

        //odometry = new HolonomicOdometry(
          //      lOdom::getDistance,
            //    rOdom::getDistance,
              //  bOdom::getDistance,
                //TRACK_WIDTH,
                //CENTER_WHEEL_OFFSET
       //);

        lfencoder.setDirection(DcMotorSimple.Direction.REVERSE);
        lbencoder.setDirection(DcMotorSimple.Direction.REVERSE);

        lfencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbencoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //telemetry.addData("pos", robotPose);
        telemetry.addData("lf", lfencoder.getCurrentPosition());
        telemetry.addData("rf", rfencoder.getCurrentPosition());
        telemetry.addData("lb", lbencoder.getCurrentPosition());
        telemetry.addData("rb", rbencoder.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        //odometry.updatePose();
        //robotPose = odometry.getPose();
        //telemetry.addData("pos", robotPose);
        //telemetry.addData("lf", lfencoder.getCurrentPosition());
        //telemetry.addData("rf", rfencoder.getCurrentPosition());
        //telemetry.addData("lb", lbencoder.getCurrentPosition());
        //telemetry.addData("rb", rbencoder.getCurrentPosition());

        telemetry.addData("imu info", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES));
        telemetry.update();
    }
}
