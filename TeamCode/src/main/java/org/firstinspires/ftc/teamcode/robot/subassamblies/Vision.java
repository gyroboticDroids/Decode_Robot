package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision {
    private final Hardware hardware;

    public Vision(Hardware h) {
        hardware = h;
        hardware.limelight.deleteSnapshots();
    }

    public Pose getRobotPosFromTarget() {
        LLResult result = hardware.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D robotPos = result.getBotpose();

            double angle = robotPos.getOrientation().getYaw(AngleUnit.DEGREES) + 90;

            if (angle > 360)
                angle -= 360;

            return new Pose(robotPos.getPosition().y / 0.0254 + 70.625, -robotPos.getPosition().x / 0.0254 + 70.625,
                    Math.toRadians(angle));
        }

        return null;
    }

    public void makeSnapshot(String name) {
        hardware.limelight.captureSnapshot(name);
    }
}
