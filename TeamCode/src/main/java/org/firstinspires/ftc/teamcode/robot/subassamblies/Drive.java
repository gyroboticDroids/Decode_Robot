package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;

public class Drive extends DriveConstants {
    private final Hardware hardware;
    private final Gamepad gamepad;
    private final Vision vision;

    private double x;
    private double y;
    private double rx;

    private double speedMultiplier = DRIVE_SPEED;

    public double headingLock = -1;
    private boolean resetHeading = false;
    private boolean prevResetHeading = false;

    private boolean park = false;

    private boolean autoDriveIsActive = false;
    private boolean prevAutoDriveIsActive = false;

    private final PIDFController turnPIDF;

    private int state = 0;
    private int prevIdentifier = 0;

    public Drive(Hardware h, Gamepad g) {
        hardware = h;
        gamepad = g;

        vision = new Vision(h);

        hardware.configureTeleop();

        hardware.follower.setStartingPose(TransferConstants.endPose);

        turnPIDF = new PIDFController(TURN_PIDF);
    }

    public void update() {
        if (!autoDriveIsActive) {
            hardware.follower.poseTracker.update();

            if (prevAutoDriveIsActive) {
                hardware.follower.breakFollowing();
                hardware.resetBrakeMode();
                state = 0;
            }

            input();

            if (resetHeading && !prevResetHeading) {
                Pose updatedPose = vision.getRobotPosFromTarget();

                if (updatedPose != null) {
                    hardware.follower.setPose(updatedPose);
                    vision.makeSnapshot(updatedPose.toString());
                    gamepad.rumble(0.5, 0.5, 500);
                } else {
                    hardware.follower.setPose(new Pose(70.625, 70.625, TransferConstants.isAllianceColorRed ? 0 : Math.toRadians(180)));
                }

                hardware.follower.poseTracker.resetOffset();
            }

            updatePark();

            if (headingLock >= 0 && !park)
                autoTurn(headingLock);

            updateMovement();
        } else {
            hardware.follower.update();
        }

        prevResetHeading = resetHeading;
        prevAutoDriveIsActive = autoDriveIsActive;
        autoDriveIsActive = false;
    }

    public void offsetRobotPos(int x, int y) {
        Pose current = hardware.follower.getPose();
        hardware.follower.poseTracker.setXOffset(current.getX() + x * (TransferConstants.isAllianceColorRed ? 1 : -1));
        hardware.follower.poseTracker.setYOffset(current.getY() + y * (TransferConstants.isAllianceColorRed ? 1 : -1));
    }

    private void input() {
        y = (TransferConstants.isAllianceColorRed ? -1 : 1) * gamepad.left_stick_y * speedMultiplier;
        x = (TransferConstants.isAllianceColorRed ? 1 : -1) * gamepad.left_stick_x * speedMultiplier;
        rx = (headingLock >= 0) ? 0 : -gamepad.right_stick_x * speedMultiplier;

        if (gamepad.dpad_down)
            park = true;
        else if (gamepad.dpad_up)
            park = false;

        resetHeading = gamepad.share;
    }

    private void autoTurn(double angle) {
        double error = angle - Math.toDegrees(hardware.follower.getHeading());

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turnPIDF.updateError(error);

        rx = MathFunctions.clamp(turnPIDF.run(), -0.4, 0.4);
    }

    private void updateMovement() {
        double botHeading = hardware.follower.getHeading();

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

    private void updatePark() {
        if (park) {
            hardware.parkLeft.setPosition(PARK_LEFT_DOWN_POS);
            hardware.parkRight.setPosition(PARK_RIGHT_DOWN_POS);

            speedMultiplier = PARK_SPEED;
        } else {
            hardware.parkLeft.setPosition(PARK_LEFT_UP_POS);
            hardware.parkRight.setPosition(PARK_RIGHT_UP_POS);

            speedMultiplier = gamepad.left_bumper ? SLOW_SPEED : DRIVE_SPEED;
        }
    }

    public void driveToGate() {
        if (prevIdentifier != 0) {
            state = 0;
            prevIdentifier = 0;
        }

        autoDriveIsActive = true;

        switch (state) {
            case 0:
                Path gateBump = new Path(new BezierLine(hardware.follower.getPose(), new Pose(115, 63, Math.toRadians(0))));
                gateBump.setLinearHeadingInterpolation(hardware.follower.getHeading(), 0);
                hardware.follower.followPath(gateBump);
                state++;
                break;

            case 1:
                if(hardware.follower.getCurrentTValue() > 0.97){
                    Path gateCollect = new Path(new BezierLine(new Pose(115, 63, Math.toRadians(0)), new Pose(130, 56, Math.toRadians(36))));
                    gateCollect.setLinearHeadingInterpolation(0, Math.toRadians(36));
                    hardware.follower.followPath(gateCollect);
                    state++;
                }
                break;
        }
    }

    public boolean isPark() {
        return park;
    }
}
