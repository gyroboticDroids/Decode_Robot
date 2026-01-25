package org.firstinspires.ftc.teamcode.robot.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

@Configurable
public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(138, 138);
    public static Pose GOAL_POS_BLUE = mirror(GOAL_POS_RED);
    public static double SCORE_HEIGHT = 29; //inches
    public static double SCORE_ANGLE = Math.toRadians(-30); //radians
    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches
    public static double SCORE_ACCURACY = 6;

    public static Pose getGoalPos() {
        return (TransferConstants.isAllianceColorRed) ? ShooterConstants.GOAL_POS_RED : ShooterConstants.GOAL_POS_BLUE;
    }

    public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(0.006, 0.0015, 0, 0.091);
    public static double FLYWHEEL_OFF = 0; //ticks per second
    public static double FLYWHEEL_ACCURACY = 70; //ticks per second
    public static double FLYWHEEL_RAMP_SPEED = 250;
    public static double FLYWHEEL_MIN_SPEED = 0;
    public static double FLYWHEEL_MAX_SPEED = 2200;

    public static PIDFCoefficients TURRET_PIDF = new PIDFCoefficients(0.015, 0, 0.0008, 0);
    public static double TURRET_F_ERROR = 3;
    public static double TURRET_GEAR_RATIO = (28.0 / 91) / (22.0 / 97);
    public static double TURRET_TICKS_PER_DEGREE = 303 / 180.0 * TURRET_GEAR_RATIO;
    public static double TURRET_RESET_POS = -93 * TURRET_TICKS_PER_DEGREE;//ticks
    public static double TURRET_MIN_ANGLE = -93;
    public static double TURRET_MAX_ANGLE = 92;
    public static double TURRET_MAX_SPEED = 0.7;

    public static double HOOD_MIN_ANGLE = Math.toRadians(74.5); //radians
    public static double HOOD_MAX_ANGLE = Math.toRadians(42.4); //radians

    public static double LAUNCHER_UP = 0.456;//
    public static double LAUNCHER_DOWN = 0.126;//

    public static double DOOR_OPEN = 0.576;//
    public static double DOOR_CLOSED = 0.428;//

    public static double BALL_DETECTION_TIME = 0.05;
    public static double BALL_DETECTION_DISTANCE = 1;//inches
    public static double flywheelOffset = 0;

    public static double getFlywheelTicksFromVelocity(double velocity) {
        double v = velocity / 12;
        return MathFunctions.clamp(8.535 * v * v - 200.28 * v + 2310 + flywheelOffset, FLYWHEEL_MIN_SPEED,
                FLYWHEEL_MAX_SPEED);
    }

    public static double getHoodTicksFromDegrees(double degrees) {
        return 0.0215 * degrees - 0.6937;
    }

    public static Pose mirror(Pose oldPose) {
        return new Pose(141.5 - oldPose.getX(), oldPose.getY(), MathFunctions.normalizeAngle(Math.PI - oldPose.getHeading()));
    }
}
