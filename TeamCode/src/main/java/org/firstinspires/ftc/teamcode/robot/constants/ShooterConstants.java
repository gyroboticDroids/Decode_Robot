package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(141, 141);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = (45 - 14.5) / 12; //feet
    public static double SCORE_ANGLE = Math.toRadians(-20); //radians

    public static Pose getGoalPos() {
        return (TransferConstants.isAllianceColorRed) ? ShooterConstants.GOAL_POS_RED : ShooterConstants.GOAL_POS_BLUE;
    }

    public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(200, 0.7, 0, 0);
    public static double FLYWHEEL_OFF = 0; //ticks per second
    public static double FLYWHEEL_ACCURACY = 60; //ticks per second
    public static double FLYWHEEL_RAMP_SPEED = 300;
    public static double FLYWHEEL_MIN_SPEED = 0;
    public static double FLYWHEEL_MAX_SPEED = 1400;

    public static com.pedropathing.control.PIDFCoefficients TURRET_PIDF =
            new com.pedropathing.control.PIDFCoefficients(0.025, 0, 0.0012, 0);
    public static double TURRET_TICKS_PER_DEGREE = 303 / 180.0;
    public static double TURRET_RESET_POS = -95 * TURRET_TICKS_PER_DEGREE;//ticks
    public static double TURRET_TRIM_SPEED = 0.5;
    public static double TURRET_MIN_ANGLE = -93;
    public static double TURRET_MAX_ANGLE = 92;
    public static double TURRET_MAX_SPEED = 0.5;

    public static double HOOD_MIN_ANGLE = 0.126;
    public static double HOOD_MAX_ANGLE = 0.904;

    public static double LAUNCHER_UP = 0.456;//
    public static double LAUNCHER_DOWN = 0.126;//

    public static double DOOR_OPEN = 0.653;//
    public static double DOOR_CLOSED = 0.452;//

    public static double BALL_DETECTION_TIME = 0.05;
    public static double BALL_DETECTION_DISTANCE = 1;//inches

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return MathFunctions.clamp(68.041 * velocity - 243.54, FLYWHEEL_MIN_SPEED, FLYWHEEL_MAX_SPEED);
    }

    public static double getHoodTicksFromDegrees(double degrees) {
        return MathFunctions.clamp(0.0268 * degrees - 0.9569, HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
    }
}
