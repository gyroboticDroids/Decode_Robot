package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Vision;

@TeleOp (name = "Vision Test")
public class VisionTurretTest extends OpMode {
    private Vision vision;

    @Override
    public void init() {
        vision = new Vision(new Hardware(hardwareMap));
    }

    @Override
    public void loop() {
        telemetry.addData("Robot position", vision.getRobotPosFromTarget());
    }
}
