package org.firstinspires.ftc.teamcode.robot.constants;

import com.pedropathing.geometry.Pose;

public class TransferConstants {
    public static int allianceColor = 0;
    public static int endTurretPos = 0;
    public static Pose endPose = new Pose(72, 72, 0);

    public static void resetConstants() {
        allianceColor = 0;
        endTurretPos = 0;
        endPose = new Pose(72, 72, 0);
    }
}
