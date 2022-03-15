package org.sbs.bears.robotframework.controllers;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.sbs.bears.robotframework.enums.IntakeState;

public interface IntakeController {
   void setState(IntakeState state);
   boolean isObjectInPayload();
   void loadItemIntoSlideForAutonomousOnly();
   public ModernRoboticsI2cRangeSensor distanceSensor = null;
}
