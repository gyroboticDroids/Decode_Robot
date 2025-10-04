package org.firstinspires.ftc.teamcode.subassamblies;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Vision {
    private final Hardware hardware;

    public Vision(Hardware h) {
        hardware = h;
    }

    public Pose2D getRobotPosFromTarget(){
        LLResult result = hardware.limelight.getLatestResult();

        if(result != null && result.isValid()) {
            Pose3D robotPos = result.getBotpose();

            return new Pose2D(DistanceUnit.INCH, robotPos.getPosition().y / 0.0254, -robotPos.getPosition().x / 0.0254,
                    AngleUnit.DEGREES, robotPos.getOrientation().getYaw() - 180);
        }

        return null;
    }
}
