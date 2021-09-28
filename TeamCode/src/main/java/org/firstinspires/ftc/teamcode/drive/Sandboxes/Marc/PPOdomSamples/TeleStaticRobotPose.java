package org.firstinspires.ftc.teamcode.drive.Sandboxes.Marc.PPOdomSamples;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "test encoderMarc")
public class TeleStaticRobotPose extends LinearOpMode {

    private MotorEx leftEncoder, rightEncoder, perpEncoder;
    private HolonomicOdometry odometry;

    //private Motor lf = new Motor(hardwareMap, "lf", Motor.GoBILDA.RPM_1150);
    //private Motor rf = new Motor(hardwareMap, "leftodom", Motor.GoBILDA.RPM_1150);
    //private Motor lb = new Motor(hardwareMap, "backodom", Motor.GoBILDA.RPM_1150);
    //private Motor rb = new Motor(hardwareMap, "rightodom", Motor.GoBILDA.RPM_1150);

    private DcMotor lf = hardwareMap.get(DcMotor.class, "lf");
    private DcMotor rf = hardwareMap.get(DcMotor.class, "leftodom");
    private DcMotor lb = hardwareMap.get(DcMotor.class, "backodom");
    private DcMotor rb = hardwareMap.get(DcMotor.class, "rightodom");


    //private Motor.Encoder leftOdom = new Motor.Encoder(hardwareMap, "leftodom", );

    private Encoder lOdom = hardwareMap.get(Encoder.class, "leftOdom");
    private Encoder rOdom = hardwareMap.get(Encoder.class, "rightOdom");
    private Encoder bOdom = hardwareMap.get(Encoder.class, "backOdom");

    private SampleMecanumDrive drive;

    public static final double TRACKWIDTH = 14.31;
    public static final double CENTER_WHEEL_OFFSET = 0.477;
    public static final double WHEEL_DIAMETER = 2.0;
    // if needed, one can add a gearing term here
    public static final double TICKS_PER_REV = 8192;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;



    @Override
    public void runOpMode() throws InterruptedException {
        //leftEncoder = new MotorEx(hardwareMap, "left odometer");
        //rightEncoder = new MotorEx(hardwareMap, "right odometer");
        //perpEncoder = new MotorEx(hardwareMap, "center odometer");

        //leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        //rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        //perpEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        //odometry = new HolonomicOdometry(
          //      leftEncoder::getDistance,
            //    rightEncoder::getDistance,
              //  perpEncoder::getDistance,
                //TRACKWIDTH,
                //CENTER_WHEEL_OFFSET
        //);

        // read the current position from the position tracker
       // odometry.updatePose(PositionTracker.robotPose);

        //drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addData("left", lOdom.getCurrentPosition());
        telemetry.addData("right", rOdom.getCurrentPosition());
        telemetry.addData("back", bOdom.getCurrentPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            lf.setPower(.3);
            rb.setPower(.3);
            // teleop things
            //drive.setWeightedDrivePower(
            //        new Pose2d(
            //                -0.3*gamepad1.left_stick_y,
            //               -0.3*gamepad1.left_stick_x,
            // -0.3*gamepad1.right_stick_x
            //        )
            //);
            // update position
            //odometry.updatePose();
            //PositionTracker.robotPose = odometry.getPose();
            telemetry.update();
        }
    }

}

