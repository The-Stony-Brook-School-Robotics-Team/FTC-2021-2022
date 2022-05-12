package org.firstinspires.ftc.teamcode.sandboxes.William;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.DriveConstantsMain;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "AAAAAAAAAAA")
public class WallFollower extends OpMode {
    double DISTANCE_TO_TRAVEL = DriveConstantsMain.inchesToEncoderTicks(4);
    static double DISTANCE_TO_WALL = 200;
    double forwardPowerOffSet = 0;

    double tP = 0;
    double tI = 0;
    double tD = 0;
    double hP = 3;
    double hI = 0.1;
    double hD = 0.2;
    PIDController tPID = new PIDController(tP, tI, tD);
    PIDController hPID = new PIDController(hP, hI, hD);

    double tPower;
    double hPower;
    double leftStickX = 0;
    double leftStickY = 0;
    double rightStickX = 0;
    DcMotor lf;
    DcMotor rf;
    DcMotor lb;
    DcMotor rb;
    ModernRoboticsI2cRangeSensor distanceSensor;
    BNO055IMU imu;
    double average = -1;
    int currentSelection = 1;
    boolean isPressingA = false;
    boolean isPressingDpadLeft = false;
    boolean isPressingDpadRight = false;
    boolean isPressingDpadUp = false;
    boolean isPressingDpadDown = false;

    @Override
    public void init() {
        tPID.setSetPoint(DISTANCE_TO_WALL);
        tPID.setTolerance(0.8, 4);
        hPID.setSetPoint(0);
        hPID.setTolerance(1, 1);
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "d1");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        distanceBefore = getDistanceSensorRead();
    }

    @Override
    public void loop() {
        update();
        drive();
        displayCurrentSelection();

        if (gamepad1.dpad_up && !isPressingDpadUp) {
            isPressingDpadUp = true;
            changeCurrentSelection(0.1);
        } else if (!gamepad1.dpad_up && isPressingDpadUp) {
            isPressingDpadUp = false;
        }

        if (gamepad1.dpad_down && !isPressingDpadDown) {
            isPressingDpadDown = true;
            changeCurrentSelection(-0.1);
        } else if (!gamepad1.dpad_down && isPressingDpadDown) {
            isPressingDpadDown = false;
        }

        if (gamepad1.dpad_left && !isPressingDpadLeft) {
            isPressingDpadLeft = true;
            currentSelection = 1 + (currentSelection + 4) % 6;
        } else if (!gamepad1.dpad_left && isPressingDpadLeft) {
            isPressingDpadLeft = false;
        }

        if (gamepad1.dpad_right && !isPressingDpadRight) {
            isPressingDpadRight = true;
            currentSelection = 1 + (currentSelection % 6);
        } else if (!gamepad1.dpad_right && isPressingDpadRight) {
            isPressingDpadRight = false;
        }

        if (gamepad1.a && !isPressingA) {
            isPressingA = true;
            if (forwardPowerOffSet == 0)
                forwardPowerOffSet = 0.4;
            else
                forwardPowerOffSet = 0;
        } else if (!gamepad1.a && isPressingA) {
            isPressingA = false;
        }
    }

    public void displayCurrentSelection() {
        String temp;
        switch (currentSelection) {
            case 1:
                temp = "tP";
                break;
            case 2:
                temp = "tI";
                break;
            case 3:
                temp = "tD";
                break;
            case 4:
                temp = "hP";
                break;
            case 5:
                temp = "hI";
                break;
            default:
                temp = "hD";
                break;
        }

        telemetry.addData("selection", temp);
        telemetry.addData("distance", distanceBefore);
        telemetry.addData("tP", tP);
        telemetry.addData("tI", tI);
        telemetry.addData("tD", tD);
        telemetry.addData("hP", hP);
        telemetry.addData("hI", hI);
        telemetry.addData("hD", hD);
        telemetry.update();
    }

    public void changeCurrentSelection(double offset) {
        switch (currentSelection) {
            case 1:
                tP += offset;
                break;
            case 2:
                tI += offset;
                break;
            case 3:
                tD += offset;
                break;
            case 4:
                hP += offset;
                break;
            case 5:
                hI += offset;
                break;
            default:
                hD += offset;
                break;
        }

        tPID = new PIDController(tP, tI, tD);
        hPID = new PIDController(hP, hI, hD);
    }

    public void drive() {
        lf.setPower(-forwardPowerOffSet + leftStickY + leftStickX + rightStickX);
        rf.setPower(forwardPowerOffSet - leftStickY + leftStickX + rightStickX);
        lb.setPower(-forwardPowerOffSet + leftStickY - leftStickX + rightStickX);
        rb.setPower(forwardPowerOffSet - leftStickY - leftStickX + rightStickX);
    }

    public double getHeadingSensorRead() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double getDistanceSensorRead() {
        return (int) distanceSensor.getDistance(DistanceUnit.MM);
    }

    public Vector2d getLeftJoyStickValues() {
        double heading = getHeadingSensorRead();
        return new Vector2d(-Math.sin(heading) * tPower, Math.cos(heading) * tPower);
    }

    double distanceBefore;
    double distanceTolerance = 400;

    public double getDistanceToWall() {
        double currentDistance = getDistanceSensorRead();
        if (Math.abs(distanceBefore - currentDistance) > distanceTolerance)
            currentDistance = distanceBefore;
        else
            distanceBefore = currentDistance;

        return currentDistance * Math.cos(getHeadingSensorRead());
    }

    public void update() {
        tPower = tPID.calculate(getDistanceToWall());
        hPower = hPID.calculate(getHeadingSensorRead());

        Vector2d leftJoyStickValues = getLeftJoyStickValues();
        leftStickX = leftJoyStickValues.getX();
        leftStickY = leftJoyStickValues.getY();
        rightStickX = hPower;

        average = (double)(Math.abs(lf.getCurrentPosition()) + Math.abs(rf.getCurrentPosition()) + Math.abs(lb.getCurrentPosition()) + Math.abs(rb.getCurrentPosition())) / 4.0;
        if(average >= DISTANCE_TO_TRAVEL){
            forwardPowerOffSet = 0;
        }
    }


}
