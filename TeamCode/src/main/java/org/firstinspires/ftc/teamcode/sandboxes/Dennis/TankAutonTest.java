package org.firstinspires.ftc.teamcode.sandboxes.Dennis;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp()
public class TankAutonTest extends LinearOpMode {

    private BNO055IMU imu;
    private DcMotorEx lf, rf, lb, rb;

    public double p = 1;
    private int i = 1;
    private int d = 1;

    private float angleTolerance = 2;

    private TankDrive tankDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        //imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //tankdrive
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        tankDrive = new TankDrive(lf, rf, lb, rb);

        waitForStart();

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float initalHeading = angles.firstAngle;

        while(!isStopRequested() && opModeIsActive()) {
            tankDrive.move(10);
            while (tankDrive.isBusy()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                float current = angles.firstAngle;
                if(current - initalHeading > 2) {
                    double bias = 0.3;
                    float err = current - initalHeading;
                    double fin1 = bias * err / p;
                    double fin2 = (bias * err * p) + 1;
                    tankDrive.applyRightMultipliers(fin1, fin2);
                } else if(current - initalHeading < -2) {
                    double bias = 0.3;
                    float err = current - initalHeading;
                    double fin1 = bias * err / p;
                    double fin2 = (bias * err / p) + 1;
                    tankDrive.applyLeftMultipliers(fin1, fin2);
                }
            }
        }
    }

}
