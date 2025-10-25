package org.firstinspires.ftc.teamcode.robot.constants;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterConstants {
    public static PIDFCoefficients PIDF = new PIDFCoefficients(300, 0, 0, 9.5);
    public static double TURN_P_GAIN = 0.022;

    public static double PARK_LEFT_DOWN_POS = 0;
    public static double PARK_RIGHT_DOWN_POS = 0;
    public static double PARK_LEFT_UP_POS = 0;
    public static double PARK_RIGHT_UP_POS = 0;

    public static double DRIVE_SPEED = 1;
    public static double PARK_SPEED = 0.5;
}
