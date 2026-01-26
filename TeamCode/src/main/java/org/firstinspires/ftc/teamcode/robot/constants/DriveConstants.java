package org.firstinspires.ftc.teamcode.robot.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.Pose;

@Configurable
public class DriveConstants {
    public static double getParkHeading() { return TransferConstants.isAllianceColorRed ? 0 : 180; }

    public static double getGateHeading() { return TransferConstants.isAllianceColorRed ? 37 : 143; }

    public static PIDFCoefficients DRIVE_PIDF = new PIDFCoefficients(0.03, 0, 0.002, 0);
    public static double DRIVE_MAX_POWER = 1;
    public static PIDFCoefficients TURN_PIDF = new PIDFCoefficients(0.022, 0, 0.001, 0);

    public static Pose gateReady = new Pose(115, 63, Math.toRadians(0)),
            gateCollect = new Pose(130, 56, Math.toRadians(36));

    public static double PARK_LEFT_DOWN_POS = 0;
    public static double PARK_RIGHT_DOWN_POS = 0;
    public static double PARK_LEFT_UP_POS = 0;
    public static double PARK_RIGHT_UP_POS = 0;

    public static double DRIVE_SPEED = 0.9;
    public static double PARK_SPEED = 0;
    public static double SLOW_SPEED = 0.5;
}
