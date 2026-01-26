package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;

public class Drive extends DriveConstants {
    private final Hardware hardware;
    private final Gamepad gamepad;
    private final PoseTracker poseTracker;
    private final Vision vision;

    public Pose robotPos;

    private Pose goalOffset = new Pose(0, 0, 0);

    private double x;
    private double y;
    private double rx;

    private double speedMultiplier = DRIVE_SPEED;

    public double headingLock = -1;
    private boolean resetHeading = false;
    private boolean prevResetHeading = false;

    private boolean park = false;

    private boolean autoDriveIsActive = false;

    private Pose targetPose = new Pose();

    private final PIDFController drivePIDF;
    private final PIDFController turnPIDF;

    private int state = 0;

    public Drive(Hardware hardware, Gamepad gamepad) {
        this.hardware = hardware;
        this.gamepad = gamepad;

        vision = new Vision(hardware);

        this.hardware.configureTeleop();

        poseTracker = hardware.poseTracker;
        poseTracker.setStartingPose(TransferConstants.endPose);

        drivePIDF = new PIDFController(DRIVE_PIDF);
        turnPIDF = new PIDFController(TURN_PIDF);
    }

    public void update() {
        poseTracker.applyOffset(goalOffset);
        poseTracker.update();
        robotPos = poseTracker.getPose();

        if (!autoDriveIsActive) {
            state = 0;
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

            updatePark();

            if (headingLock >= 0 && !park)
                autoTurn(headingLock);
        } else {
            autoDriveUpdate();
        }

        updateMovement();

        prevResetHeading = resetHeading;
        autoDriveIsActive = false;
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
        double error = angle - Math.toDegrees(robotPos.getHeading());

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turnPIDF.updateError(error);

        rx = MathFunctions.clamp(turnPIDF.run(), -0.4, 0.4);
    }

    private void updateMovement() {
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

    public void setGoalOffset(Pose pose) {
        goalOffset = pose;
    }

    public void driveToGate() {
        autoDriveIsActive = true;

        switch (state) {
            case 0:
                driveToPose(gateReady);
                if(robotPos.getY() < gateReady.getY() + 2) {
                    state++;
                }
                break;

            case 1:
                driveToPose(gateCollect);
                state++;
                break;
        }
    }

    public void driveToPose(Pose pose) {
        targetPose = pose;
    }

    private void autoDriveUpdate() {
        Vector lineToPointTotal = new Vector(robotPos.minus(targetPose));

        drivePIDF.updateError(lineToPointTotal.getMagnitude());

        double motorPower = MathFunctions.clamp(drivePIDF.run(), -DRIVE_MAX_POWER, DRIVE_MAX_POWER);

        x = Math.sin(lineToPointTotal.getTheta()) * motorPower;
        y = -Math.cos(lineToPointTotal.getTheta()) * motorPower;

        autoTurn(Math.toDegrees(targetPose.getHeading()));
    }

    public boolean isPark() {
        return park;
    }
}
