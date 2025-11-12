package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(141, 141);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();

    public static Pose GOAL_POS = (TransferConstants.isAllianceColorRed) ?
            ShooterConstants.GOAL_POS_RED : ShooterConstants.GOAL_POS_BLUE;

    public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(300, 0, 0, 10);
    public static double FLYWHEEL_OFF = 0; //ticks per second
    public static double FLYWHEEL_ACCURACY = 50; //ticks per second
    public static double FLYWHEEL_TRIM_SPEED = 1;

    public static double TURRET_P_GAIN = 0.017;
    public static double TURRET_TICKS_PER_DEGREE = 1.55556;
    public static int TURRET_RESET_POS = 180 * (int)TURRET_TICKS_PER_DEGREE;//ticks
    public static double TURRET_TRIM_SPEED = 0.5;
    public static double TURRET_MIN_ANGLE = -90;
    public static double TURRET_MAX_ANGLE = 90;
    public static double TURRET_MAX_SPEED = 0.6;

    public static double HOOD_TRIM_SPEED = 0.001;

    public static double LAUNCHER_UP = 0.456;//
    public static double LAUNCHER_DOWN = 0.126;//

    public static double DOOR_OPEN = 0.125;//
    public static double DOOR_CLOSED = 0.66;//

    public static double BALL_DETECTION_TIME = 0.08;
    public static double BALL_DETECTION_DISTANCE = 1.5;//inches

    private static double flywheelOffset = 0;
    private static double hoodOffset = 0;

    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(539.1203 * Math.pow(1.00531, goalDist) + flywheelOffset, 0, 1300);
    }

    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(-9.24037e-7 * Math.pow(goalDist, 3) +
                0.000267701 * Math.pow(goalDist, 2) - 0.0281248 * goalDist + 1.28255 + hoodOffset, 0.02, 0.769);
    }

    public static double launchTime(double goalDist) {
        return MathFunctions.clamp(0.00356061 * goalDist + 0.45, 0, 1.5);
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
