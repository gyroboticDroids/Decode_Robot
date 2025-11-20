package org.firstinspires.ftc.teamcode.robot.teaching;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;

@TeleOp(name = "drive with auto turn")
public class RobotAutoTurn extends OpMode {
    private Hardware hardware;

    private double headingSetpoint = 0;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);
        hardware.configureTeleop();
    }

    @Override
    public void loop() {
        //Gets heading of robot
        hardware.poseTracker.update();
        double botHeading = hardware.poseTracker.getPose().getHeading();

        //Gets driver input
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        //Auto turning

        //Boolean to store if we want to auto turn
        boolean runAutoTurn;

        //Gets target setpoint from input
        if (gamepad1.cross) {
            headingSetpoint = 45;
            runAutoTurn = true;
        }else if (gamepad1.circle) {
            headingSetpoint = 270;
            runAutoTurn = true;
        } else {
            runAutoTurn = false;
        }

        //If input was received, turn robot to desired position
        if(runAutoTurn) {
            double error = headingSetpoint - Math.toDegrees(botHeading);

            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }

            rx = 0.022 * error;
            rx = Math.min(Math.max(rx, -0.4), 0.4);
        }

        //Field oriented mecanum drive controller
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX - rx) / denominator;
        double leftRearPower = (rotY - rotX - rx) / denominator;
        double rightFrontPower = (rotY - rotX + rx) / denominator;
        double rightRearPower = (rotY + rotX + rx) / denominator;

        hardware.leftFront.setPower(leftFrontPower);
        hardware.leftRear.setPower(leftRearPower);
        hardware.rightFront.setPower(rightFrontPower);
        hardware.rightRear.setPower(rightRearPower);
    }
}
