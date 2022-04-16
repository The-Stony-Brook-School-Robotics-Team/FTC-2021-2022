package org.sbs.bears.robotframework.controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.teleop.misc.Beta;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.sbs.bears.robotframework.Sleep;
import org.sbs.bears.robotframework.enums.DrivingMode;
import org.sbs.bears.robotframework.enums.MotorName;
import org.tensorflow.lite.task.text.qa.QaAnswer;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

public class DrivingControllerTank {
    SampleTankDrive RR;
    DrivingMode mode;
    AtomicReference<Boolean> isFollowingTraj = new AtomicReference<>();
    Map<MotorName, DcMotorEx> motorMap = new HashMap<>();
    Map<MotorName, Integer> encoderMap = new HashMap<>();
    public DrivingControllerTank(HardwareMap hardwareMap)
    {
        RR = new SampleTankDrive(hardwareMap);
        mode  = DrivingMode.STOPPED;
        isFollowingTraj.set(false);
        motorMap.clear();
        motorMap.put(MotorName.LF,hardwareMap.get(DcMotorEx.class,"lf"));
        motorMap.put(MotorName.RF,hardwareMap.get(DcMotorEx.class,"rf"));
        motorMap.put(MotorName.LB,hardwareMap.get(DcMotorEx.class,"lb"));
        motorMap.put(MotorName.RB,hardwareMap.get(DcMotorEx.class,"rb"));
    }
    public void setPos(Pose2d newPos)
    {
        RR.setPoseEstimate(newPos);
    }
    public Pose2d getPos()
    {
        return RR.getPoseEstimate();
    }
    @Beta
    public void goForwardAsync(double distance)
    {
        new Thread(()->{
            isFollowingTraj.set(true);
            updateEncoders();
            Map<MotorName,Integer> targetEncoderCounts = new HashMap<>();

        }).start();
    }
    public void goForward(double distance)
    {
        goForwardAsync(distance);
        waitForTrajToFinish();
    }

    public void waitForTrajToFinish() {
        while(isFollowingTraj.get())
        {
            Sleep.sleep(10);
        }
    }


    private void updateEncoders()
    {
        encoderMap.clear();
        encoderMap.put(MotorName.LF,motorMap.get(MotorName.LF).getCurrentPosition());
        encoderMap.put(MotorName.RF,motorMap.get(MotorName.RF).getCurrentPosition());
        encoderMap.put(MotorName.RB,motorMap.get(MotorName.RB).getCurrentPosition());
        encoderMap.put(MotorName.LB,motorMap.get(MotorName.LB).getCurrentPosition());
    }

}
