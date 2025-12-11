package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(141, 141);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();

    public static Pose getGoalPos() {
        return (TransferConstants.isAllianceColorRed) ? ShooterConstants.GOAL_POS_RED : ShooterConstants.GOAL_POS_BLUE;
    }

    public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(200, 0.7, 0, 0);
    public static double FLYWHEEL_OFF = 0; //ticks per second
    public static double FLYWHEEL_ACCURACY = 70; //ticks per second
    public static double FLYWHEEL_TRIM_SPEED = 1;
    public static double FLYWHEEL_RAMP_SPEED = 250;

    public static com.pedropathing.control.PIDFCoefficients TURRET_PIDF =
            new com.pedropathing.control.PIDFCoefficients(0.026, 0, 0.001, 0);
    public static double TURRET_TICKS_PER_DEGREE = 303 / 180.0;
    public static double TURRET_RESET_POS = -95 * TURRET_TICKS_PER_DEGREE;//ticks
    public static double TURRET_TRIM_SPEED = 0.5;
    public static double TURRET_MIN_ANGLE = -93;
    public static double TURRET_MAX_ANGLE = 92;
    public static double TURRET_MAX_SPEED = 0.5;

    public static double HOOD_TRIM_SPEED = 0.001;

    public static double LAUNCHER_UP = 0.456;//
    public static double LAUNCHER_DOWN = 0.126;//

    public static double DOOR_OPEN = 0.653;//
    public static double DOOR_CLOSED = 0.452;//

    public static double BALL_DETECTION_TIME = 0.05;
    public static double BALL_DETECTION_DISTANCE = 1;//inches

    private static double flywheelOffset = 0;
    private static double hoodOffset = 0;

    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(0.0204772 * Math.pow(goalDist, 2) + 0.643162 * goalDist + 712.90909, 0, 1400)
                + flywheelOffset;
    }

    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(-2.34831e-7 * Math.pow(goalDist, 3) + 0.0000936893 * Math.pow(goalDist, 2) - 0.0165033
                * goalDist + 1.25724, 0.11, 0.904) + hoodOffset;
    }

    public static double launchTime(double goalDist) {
        return 0.000018211 * Math.pow(goalDist, 2) - 0.00173368 * goalDist + 0.713636;
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

    public static double VELOCITY_TIME_MULTIPLIER = 0;
}
