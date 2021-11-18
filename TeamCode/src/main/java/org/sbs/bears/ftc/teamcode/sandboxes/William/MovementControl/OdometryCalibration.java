package org.firstinspires.ftc.teamcode.sandboxes.William.MovementControl;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends OpMode {
    //Drive motors
    DcMotor right_front = hardwareMap.dcMotor.get(ControllerHelper.rightFrontMotorName);
    DcMotor right_back = hardwareMap.dcMotor.get(ControllerHelper.rightBackMotorName);
    DcMotor left_front = hardwareMap.dcMotor.get(ControllerHelper.leftFrontMotorName);
    DcMotor left_back = hardwareMap.dcMotor.get(ControllerHelper.leftBackMotorName);

    //Odometry Wheels
    DcMotor verticalLeft = hardwareMap.dcMotor.get(ControllerHelper.leftOdometryName);
    DcMotor verticalRight = hardwareMap.dcMotor.get(ControllerHelper.rightOdometryName);
    DcMotor horizontal = hardwareMap.dcMotor.get(ControllerHelper.backOdometryName);

    //IMU Sensor
    BNO055IMU imu;

    final double PIVOT_SPEED = 0.5;

    //The amount of encoder ticks for each inch the robot moves. NEEDS UPDATE.
    final double COUNTS_PER_INCH = ControllerHelper.COUNTS_PER_INCH;

    double horizontalTickOffset = 0;

    private boolean hasDoneCalculation = false;

    @Override
    public void init() {
        //Initialize IMU hardware map value.
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        setPowerAll(0, 0, 0, 0);

        //Odometry System Calibration Init Complete
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!hasDoneCalculation) {
            //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
            if (getZAngle() < 90) {
                if (getZAngle() < 60) {
                    setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
                } else {
                    setPowerAll(-PIVOT_SPEED / 2, -PIVOT_SPEED / 2, PIVOT_SPEED / 2, PIVOT_SPEED / 2);
                }
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.update();
            } else {
                //Record IMU and encoder values to calculate the constants for the global position algorithm
                double angle = getZAngle();

                double encoderDifference = Math.abs(verticalLeft.getCurrentPosition()) - (Math.abs(verticalRight.getCurrentPosition()));

                double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;
                double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * COUNTS_PER_INCH);
                horizontalTickOffset = horizontal.getCurrentPosition() / Math.toRadians(getZAngle());

                telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
                //Display calculated constants
                telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
                telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

                //Display raw values
                telemetry.addData("IMU Angle", getZAngle());
                telemetry.addData("Vertical Left Position", -verticalLeft.getCurrentPosition());
                telemetry.addData("Vertical Right Position", verticalRight.getCurrentPosition());
                telemetry.addData("Horizontal Position", horizontal.getCurrentPosition());
                telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

                //Update values
                telemetry.update();
                hasDoneCalculation = true;
            }
        }
    }

    /**
     * Gets the orientation of the robot using the REV IMU
     *
     * @return the angle of the robot
     */
    private double getZAngle() {
        return (-180 * Math.PI / imu.getAngularOrientation().firstAngle);
    }

    /**
     * Sets power to all four drive motors
     */
    private void setPowerAll(double rf, double rb, double lf, double lb) {
        right_front.setPower(rf);
        right_back.setPower(rb);
        left_front.setPower(lf);
        left_back.setPower(lb);
    }
}
