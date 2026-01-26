package org.firstinspires.ftc.teamcode.robot.teleop;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Drive;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Intake;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Shooter;

import java.util.List;

@TeleOp(name = "Master Tele-op", group = "teleop")
public class MasterTeleop extends OpMode {
    private Hardware hardware;

    private Drive drive;
    private Intake intake;
    private Shooter shooter;

    private Intake.State prevIntakeState;
    private Shooter.State prevShooterState;

    private Gamepad lastGamepad1;
    private Gamepad lastGamepad2;

    //Bulk reading
    List<LynxModule> allHubs;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);

        drive = new Drive(hardware, gamepad1);
        intake = new Intake(hardware);
        shooter = new Shooter(hardware);

        //Bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        gamepad1.setLedColor(128.0 / 255, 0, 1, -1);

        lastGamepad1 = new Gamepad();
        lastGamepad2 = new Gamepad();
    }

    @Override
    public void start() {
        intake.setState(Intake.State.INTAKE);
        shooter.setState(Shooter.State.OFF);
        shooter.resetPIDFS();

        prevIntakeState = intake.getState();
        prevShooterState = shooter.getState();

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

    @Override
    public void stop() {
        TransferConstants.resetConstants();
        hardware.limelight.stop();
    }

    @Override
    public void loop() {
        //Bulk reading
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        driveUpdate();
        intakeUpdate();
        shooterUpdate();

        drive.update();
        intake.update();
        shooter.update();

        telemetryUpdate();

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

    private void telemetryUpdate() {
        telemetry.addLine("----------current draw-------");
        telemetry.addData("flywheel1", "%.2f", hardware.flywheel.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("flywheel2", "%.2f", hardware.flywheel2.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("intake", "%.2f", hardware.intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("turret", "%.2f", hardware.turret.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("dt added up", "%.2f", hardware.leftFront.getCurrent(CurrentUnit.AMPS) +
                hardware.leftRear.getCurrent(CurrentUnit.AMPS) + hardware.rightFront.getCurrent(CurrentUnit.AMPS) +
                hardware.rightRear.getCurrent(CurrentUnit.AMPS));
        telemetry.addLine("----------drive--------------");
        telemetry.addData("robot position", hardware.poseTracker.getPose());
        telemetry.addData("is heading lock", drive.headingLock);
        telemetry.addLine("robot velocity: " + hardware.poseTracker.getVelocity().getMagnitude()
                + ", angular velocity: " + hardware.poseTracker.getAngularVelocity());
        telemetry.addLine("----------intake-------------");
        telemetry.addData("state", intake.getState());
        telemetry.addData("is busy", intake.isBusy());
        telemetry.addLine("----------shooter------------");
        telemetry.addData("state", shooter.getState());
        telemetry.addData("is busy", shooter.isBusy());
        telemetry.addData("goal dist", shooter.getGoalVector().getMagnitude());
        telemetry.addData("turret reset offset", shooter.getTurretReset());
        telemetry.addData("turret pos (degrees)", hardware.turret.getCurrentPosition()
                / ShooterConstants.TURRET_TICKS_PER_DEGREE);
        telemetry.addData("turret target pos (degrees)", shooter.targetPos);
        telemetry.addData("turret pos (ticks)", hardware.turret.getCurrentPosition());
        telemetry.addData("hood angle", Math.toDegrees(shooter.getHoodAngle()));
        telemetry.addData("hood ticks", hardware.hood.getPosition());
        telemetry.addData("flywheel speed setpoint", ShooterConstants.getFlywheelTicksFromVelocity(shooter.getFlywheelSpeed()));
        telemetry.addData("flywheel speed (ticks per second)", hardware.flywheel.getVelocity());
        telemetry.addData("shooter ready", shooter.isGoalTargeted());
        telemetry.update();
    }

    private void driveUpdate() {
        drive.setGoalOffset(new Pose(((gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0)) *
                (TransferConstants.isAllianceColorRed ? 1 : -1), ((gamepad2.dpad_left ? 1 : 0) -
                (gamepad2.dpad_right ? 1 : 0)) * (TransferConstants.isAllianceColorRed ? 1 : -1)));

        if (!drive.isPark()) {
            if (gamepad1.cross) {
                drive.headingLock = DriveConstants.getParkHeading();
            } else if (gamepad1.square) {
                drive.driveToGate();
            } else {
                drive.headingLock = -1;
            }
        }

        if (gamepad1.touchpad && !lastGamepad1.touchpad && TransferConstants.isAllianceColorRed) {
            TransferConstants.isAllianceColorRed = false;
        } else if (gamepad1.touchpad && !lastGamepad1.touchpad && !TransferConstants.isAllianceColorRed) {
            TransferConstants.isAllianceColorRed = true;
        }

        gamepad1.setLedColor(TransferConstants.isAllianceColorRed ? 1 : 0, 0,
                TransferConstants.isAllianceColorRed ? 0 : 1, -1);
    }

    private void intakeUpdate() {
        if (!intake.isBusy()) {
            if (drive.isPark()) {
                intake.setState(Intake.State.INTAKE_UP);
            } else if (gamepad1.triangle || gamepad2.cross) {
                intake.setState(Intake.State.CLEAR);
            } else if (gamepad1.triangleWasReleased() || gamepad2.crossWasReleased()) {
                intake.setState(Intake.State.INTAKE);
            } else if (shooter.getState() == Shooter.State.LAUNCH) {
                intake.setState(Intake.State.INTAKE_LAUNCH);
            } else if ((gamepad1.circle && !lastGamepad1.circle || gamepad2.circle && !lastGamepad2.circle
                    || prevIntakeState == Intake.State.INTAKE_LAUNCH) && prevIntakeState != Intake.State.INTAKE) {
                intake.setState(Intake.State.INTAKE);
            } else if ((gamepad1.circle && !lastGamepad1.circle || gamepad2.circle && !lastGamepad2.circle
                    || shooter.areBallsCollected() && shooter.getState() != Shooter.State.LAUNCH)
                    && prevIntakeState == Intake.State.INTAKE) {
                intake.setState(Intake.State.INTAKE_UP);
            }
        }

        prevIntakeState = intake.getState();
    }

    private void shooterUpdate() {
        if (!shooter.isBusy()) {
            if (drive.isPark()) {
                shooter.setState(Shooter.State.OFF);
            } else if ((gamepad1.right_bumper || gamepad2.right_bumper || prevShooterState == Shooter.State.LAUNCH
                    || prevShooterState == Shooter.State.RESET || prevShooterState == Shooter.State.OFF)
                    && prevShooterState != Shooter.State.READY) {
                shooter.setState(Shooter.State.READY);
            } else if ((gamepad1.right_bumper || gamepad2.right_bumper) && prevShooterState == Shooter.State.READY
                    && shooter.isGoalTargeted()) {
                shooter.setState(Shooter.State.LAUNCH);
            } else if (gamepad2.share) {
                shooter.setState(Shooter.State.RESET);
            }
        }

        if (gamepad2.options && !lastGamepad2.options && shooter.runTurret) {
            shooter.runTurret = false;
            gamepad2.rumble(0.5, 0.5, 500);
            gamepad2.setLedColor(1, 0, 0, -1);
        } else if (gamepad2.options && !lastGamepad2.options && !shooter.runTurret) {
            shooter.runTurret = true;
            gamepad2.rumble(0.5, 0.5, 500);
            gamepad2.setLedColor(0, 1, 0, -1);
        }

        prevShooterState = shooter.getState();
    }
}
