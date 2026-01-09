package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.geometry.Pose;

public class TransferConstants {
    public static boolean isAllianceColorRed = true;
    public static int endTurretPos = 0;
    public static Pose endPose = new Pose(70.625, 70.625,0);

    public static void resetConstants() {
        isAllianceColorRed = true;
        endTurretPos = 0;
        endPose = new Pose(70.625, 70.625, 0);
    }
}
