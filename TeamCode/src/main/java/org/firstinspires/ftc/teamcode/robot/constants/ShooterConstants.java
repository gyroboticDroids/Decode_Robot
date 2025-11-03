package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(65, 70, 0);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();

    public static Pose GOAL_POS = (TransferConstants.isAllianceColorRed) ?
            ShooterConstants.GOAL_POS_RED : ShooterConstants.GOAL_POS_BLUE;

    public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(300, 0, 0, 9.5);
    public static double FLYWHEEL_OFF = 0; //ticks per second
    public static double FLYWHEEL_REJECT = 500; //ticks per second
    public static double FLYWHEEL_ACCURACY = 100; //ticks per second
    public static double FLYWHEEL_TRIM_SPEED = 0.001;

    public static double TURRET_P_GAIN = 0.005;
    public static double TURRET_TICKS_PER_DEGREE = 100;
    public static int TURRET_RESET_POS = 1000;//ticks
    public static double TURRET_TRIM_SPEED = 0.001;

    public static double HOOD_TRIM_SPEED = 0.001;

    public static double LAUNCHER_UP = 0;
    public static double LAUNCHER_DOWN = 0;

    public static double DOOR_OPEN = 0;
    public static double DOOR_CLOSED = 0;

    private static double flywheelOffset = 0;
    private static double hoodOffset = 0;

    public static double flywheelSpeed(double goalDist) {
        return goalDist + flywheelOffset;
    }

    public static double hoodAngle(double goalDist) {
        return 1 / goalDist + hoodOffset;
    }

    public static double launchTime(double goalDist) {
        return goalDist;
    }

    public static void flywheelOffset(double f) {
        flywheelOffset += f;
    }

    public static void hoodOffset(double h) {
        hoodOffset += h;
    }

    public static double getHoodOffset() {
        return hoodOffset;
    }

    public static double getFlywheelOffset() {
        return flywheelOffset;
    }
}
