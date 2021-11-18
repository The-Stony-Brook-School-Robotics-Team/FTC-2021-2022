package org.firstinspires.ftc.teamcode.sandboxes.Max;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//This is the code of Max
@TeleOp(name="KEHANSandBox", group="drive")
public class KEHANSandBox extends OpMode {
    private static BNO055IMU imu;
    private static DcMotor lf = null;
    private static DcMotor rf = null;
    private static DcMotor lb = null;
    private static DcMotor rb = null;
    boolean pressingX = false;
    boolean pressingB = false;
    boolean pressingA = false;
    boolean pressingY = false;
    boolean pressingRightBumper = false;
    boolean pressingLeftBumper = false;
    boolean pressingRightDpad = false;
    boolean pressingLeftDpad = false;
    boolean pressingUp = false;
    boolean pressingDown = false;


    private SampleMecanumDrive drive;


    @Override
    public void init() {
        //Motor Setup
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "leftodom");
        lb = hardwareMap.get(DcMotor.class, "backodom");
        rb = hardwareMap.get(DcMotor.class, "rightodom");
        AxesOrder axesOrder = AxesOrder.XYZ;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Pose2d startPose = new Pose2d(-61, -27, 0);
        //Pose2d startPose = new Pose2d(12, -6, 0);
        //drive.setPoseEstimate(startPose);
    }

    @Override
    public void loop() {






        drive.setWeightedDrivePower(
                new Pose2d(
                        -0.3*gamepad1.left_stick_y,
                        -0.3*gamepad1.left_stick_x,
                        -0.3*gamepad1.right_stick_x
                )
        );
        if (!pressingA && gamepad1.a) {
            //goForward();
            rotate(90);
            pressingA = true;
        }
        else if (pressingA && !gamepad1.a) {
            pressingA = false;
        }
    }

    public void goForward() {
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d())
                .forward(40)
                .build());
        drive.update();
    }

    public static void rotate(double DestinedAngle){

    //P
    Orientation current_axis = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    double CurrentAngle = current_axis.firstAngle;
    while(DestinedAngle - CurrentAngle > 1 || DestinedAngle - CurrentAngle < -1){
        double p_power = DestinedAngle - CurrentAngle;
        double p_magnifier = 1.3;
        //D
    AngularVelocity previous_axis = imu.getAngularVelocity();
        double d_magifier = 3;
        rf.setPower(p_magnifier * p_power);
        lb.setPower(-(p_magnifier * p_power));
        previous_axis = imu.getAngularVelocity();
        double PreviousAngle = CurrentAngle - previous_axis.xRotationRate;
        double d_power = CurrentAngle - PreviousAngle;
        rf.setPower(p_magnifier * p_power + d_magifier * d_power);
        lb.setPower(-(p_magnifier * p_power + d_magifier * d_power));
    }
    rf.setPower(0);
    lb.setPower(0);

    }

}
