package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstants {
    public static Pose GOAL_POS = new Pose(65, 70, 0);

    public static PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(300, 0, 0, 9.5);
    public static double TURRET_P_GAIN = 0.01;

    public static double flywheelSpeed(double goalDist) {
        return goalDist;
    }

    public static double hoodAngle(double goalDist) {
        return 1 / goalDist;
    }
}
