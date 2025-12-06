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

    public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(300, 0.8, 0, 0);
    public static double FLYWHEEL_OFF = 0; //ticks per second
    public static double FLYWHEEL_ACCURACY = 50; //ticks per second
    public static double FLYWHEEL_TRIM_SPEED = 1;

    public static double TURRET_P_GAIN = 0.023;
    public static double TURRET_TICKS_PER_DEGREE = 1.55556;
    public static double TURRET_RESET_POS = -91 * TURRET_TICKS_PER_DEGREE;//ticks
    public static double TURRET_TRIM_SPEED = 0.5;
    public static double TURRET_MIN_ANGLE = -91;
    public static double TURRET_MAX_ANGLE = 91;
    public static double TURRET_MAX_SPEED = 0.6;

    public static double HOOD_TRIM_SPEED = 0.001;

    public static double LAUNCHER_UP = 0.456;//
    public static double LAUNCHER_DOWN = 0.126;//

    public static double DOOR_OPEN = 0.125;//
    public static double DOOR_CLOSED = 0.66;//

    public static double BALL_DETECTION_TIME = 0.05;
    public static double BALL_DETECTION_DISTANCE = 1;//inches

    private static double flywheelOffset = 0;
    private static double hoodOffset = 0;

    public static double flywheelSpeed(double goalDist) {
        return MathFunctions.clamp(554.24722 * Math.pow(1.00505, goalDist) + 280, 0, 1300) + flywheelOffset;
    }

    public static double hoodAngle(double goalDist) {
        return MathFunctions.clamp(-8.2635e-7 * Math.pow(goalDist, 3) +
                0.000261473 * Math.pow(goalDist, 2) - 0.0283785 * goalDist + 1.29433, 0.2, 0.769) + hoodOffset;
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

    public static double FLYWHEEL_TPS_TO_VELOCITY = 40;
    public static double HOOD_TICKS_TO_DEGREES = 0.5 / 40;
}
