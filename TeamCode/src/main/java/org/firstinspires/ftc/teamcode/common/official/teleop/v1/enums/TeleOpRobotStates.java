package org.firstinspires.ftc.teamcode.common.official.teleop.v1.enums;

public enum TeleOpRobotStates {
    STOPPED(0), INITIALIZING(1), RUNNING(3), AUTONOMOUS(4), DEBUG(5);
    private int ident;
    TeleOpRobotStates(int ident) {
        this.ident = ident;
    }
}