package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;

public class Drive {
    private final Hardware hardware;
    private final Gamepad gamepad;
    private final PoseTracker poseTracker;
    private final Vision vision;

    Pose robotPos;

    private double x;
    private double y;
    private double rx;

    private double speedMultiplier = DriveConstants.DRIVE_SPEED;

    public double headingLock = -1;
    private boolean resetHeading = false;
    private boolean prevResetHeading = false;

    private boolean park = false;

    public Drive(Hardware hardware, Gamepad gamepad) {
        this.hardware = hardware;
        this.gamepad = gamepad;

        vision = new Vision(hardware);

        this.hardware.configureTeleop();

        poseTracker = hardware.poseTracker;
        poseTracker.setStartingPose(TransferConstants.endPose);
    }

    public void update() {
        input();

        if (resetHeading && !prevResetHeading) {
            Pose updatedPose = vision.getRobotPosFromTarget();

            if (updatedPose != null) {
                poseTracker.setPose(updatedPose);
                vision.makeSnapshot(updatedPose.toString());
                gamepad.rumble(0.5, 0.5, 500);
            } else {
                poseTracker.setPose(new Pose(70.625, 70.625, TransferConstants.isAllianceColorRed ? 0 : Math.toRadians(180)));
            }
        }

        poseTracker.update();
        robotPos = poseTracker.getPose();

        if (headingLock >= 0)
            autoTurn(headingLock);

        park();
        movement();

        prevResetHeading = resetHeading;
    }

    private void input() {
        y = (TransferConstants.isAllianceColorRed ? -1 : 1) * gamepad.left_stick_y * speedMultiplier;
        x = (TransferConstants.isAllianceColorRed ? 1 : -1) * gamepad.left_stick_x * speedMultiplier;
        rx = (headingLock >= 0) ? 0 : -gamepad.right_stick_x;

        if (gamepad.dpad_down)
            park = true;
        else if (gamepad.dpad_up)
            park = false;

        resetHeading = gamepad.share;
    }

    private void autoTurn(double angle) {
        double error = angle - Math.toDegrees(robotPos.getHeading());

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        rx = DriveConstants.TURN_P_GAIN * error;
        rx = Math.min(Math.max(rx, -0.4), 0.4);
    }

    private void movement() {
        double botHeading = robotPos.getHeading();

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
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

    private void park() {
        if (park) {
            hardware.parkLeft.setPosition(DriveConstants.PARK_LEFT_DOWN_POS);
            hardware.parkRight.setPosition(DriveConstants.PARK_RIGHT_DOWN_POS);

            speedMultiplier = DriveConstants.PARK_SPEED;
        } else {
            hardware.parkLeft.setPosition(DriveConstants.PARK_LEFT_UP_POS);
            hardware.parkRight.setPosition(DriveConstants.PARK_RIGHT_UP_POS);

            speedMultiplier = gamepad.left_bumper ? DriveConstants.SLOW_SPEED : DriveConstants.DRIVE_SPEED;
        }
    }

    public boolean isPark() {
        return park;
    }
}
