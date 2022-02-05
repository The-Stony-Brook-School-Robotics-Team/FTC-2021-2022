package org.firstinspires.ftc.teamcode.sandboxes.Michael.Unsafe;

import com.coyote.framework.core.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.sbs.bears.robotframework.controllers.SlideController;

@TeleOp(name="increment", group="Linear Opmode")
public class Increment extends LinearOpMode {
    private boolean qDR, qDL,qX,qY;
    private SampleMecanumDrive drive;
    private SlideController slideCtrl;
    //private IntakeController frontIntake;

    enum state {
        TO_EXT,
        TO_DROP,
        TO_RETRACT
    }

    state state2 = state.TO_EXT;
    public void runOpMode() throws InterruptedException {
        //DcMotor compliantWheel = hardwareMap.get(DcMotor.class, "motor");
        //Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "2m");
        slideCtrl = new SlideController(hardwareMap,telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        //scooper.setDirection(Servo.Direction.REVERSE);
        //compliantWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        double pos = .2;
        boolean pressingB = false;
        boolean pressingA = false;
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.dpad_right && !qDR)
            {
                qDR = true;
                slideCtrl.slideMotorPowerMoving += 0.1;
                slideCtrl.slideMotorPowerMovingBack += 0.1;
            }
            if(!gamepad1.dpad_right && qDR)
            {
                qDR = false;
            }
            if(gamepad1.dpad_left && !qDL)
            {
                qDL = true;
                slideCtrl.slideMotorPowerMoving -= 0.1;
                slideCtrl.slideMotorPowerMovingBack -= 0.1;
            }
            if(!gamepad1.dpad_left && qDL)
            {
                qDL = false;
            }

            if(gamepad1.x && !qX)
            {
                qX = true;
                drive.setWeightedDrivePower(new Pose2d()); // stop robot
                Pose2d currentPos = drive.getPoseEstimate();
                Pose2d targetPos = new Pose2d(currentPos.getX()-8.42,currentPos.getY()-1.03,currentPos.getHeading() - Math.toRadians(52));
                drive.followTrajectory(drive.trajectoryBuilder(currentPos)
                        .lineToSplineHeading(targetPos)
                        .build());

                /*switch(state2)
                {
                    case TO_EXT:
                        slideCtrl.extendSlide();
                        state2 = state.TO_DROP;
                        break;
                    case TO_DROP:
                        slideCtrl.dropCube();
                        state2 = state.TO_RETRACT;
                        break;
                    case TO_RETRACT:
                        slideCtrl.retractSlide();
                        state2 = state.TO_EXT;
                        break;

                }*/
            }
            if(!gamepad1.x && qX)
            {
                qX = false;
            }






            if(gamepad1.dpad_up && !pressingA){
                pressingA = true;}
            if(!gamepad1.dpad_up && pressingA){
                pos+=.1;
                slideCtrl.verticalServo.setPosition(pos);
                pressingA = false;
            }
            if(gamepad1.dpad_down && !pressingB){
                pressingB = true;}
            if(!gamepad1.dpad_down && pressingB){
                pos-=.1;
                slideCtrl.verticalServo.setPosition(pos);
                pressingB = false;
            }
            if(gamepad1.a){
                for(double i = slideCtrl.verticalServo.getPosition(); i < 1; i+=0.001){
                    slideCtrl.verticalServo.setPosition(Range.clip(i, 0, 1));
                    try {
                        Thread.sleep(1);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
            if(gamepad1.b){
                slideCtrl.verticalServo.setPosition(0);
            }
//BASE .4 DUMP .87  PARK .787

            telemetry.addData("Posiiton: ", slideCtrl.verticalServo.getPosition());
            telemetry.addData("MotorPow: ", slideCtrl.slideMotorPowerMoving);
            telemetry.addData("SlidePos: ", slideCtrl.slideMotor.getCurrentPosition());

            telemetry.update();
        }


    }
}
