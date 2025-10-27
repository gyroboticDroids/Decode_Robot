package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision {
    private final Hardware hardware;

    public Vision(Hardware h) {
        hardware = h;
    }

    public Pose getRobotPosFromTarget() {
        LLResult result = hardware.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D robotPos = result.getBotpose();

            return new Pose(robotPos.getPosition().y / 0.0254, -robotPos.getPosition().x / 0.0254, -robotPos.getOrientation().getYaw());
        }

        return null;
    }
}
