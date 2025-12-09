package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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

        prevIntakeState = intake.getState();
        prevShooterState = shooter.getState();

        gamepad2.setLedColor(shooter.velComp ? 0 : 1, shooter.velComp ? 1 : 0, 0, -1);

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

    @Override
    public void stop() {
        TransferConstants.resetConstants();
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
        telemetry.addData("gpad test", shooter.getTurretOffset());
        telemetry.addLine("----------important----------");
        telemetry.addData("turret offset", shooter.getTurretOffset());
        telemetry.addData("hood offset", ShooterConstants.getHoodOffset());
        telemetry.addData("flywheel offset", ShooterConstants.getFlywheelOffset());
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
        telemetry.addData("goal dist", shooter.getGoalVector());
        telemetry.addData("turret reset offset", shooter.getTurretReset());
        telemetry.addData("turret pos (degrees)", hardware.turret.getCurrentPosition()
                / ShooterConstants.TURRET_TICKS_PER_DEGREE);
        telemetry.addData("turret pos (ticks)", hardware.turret.getCurrentPosition());
        telemetry.addData("hood angle", hardware.hood.getPosition());
        telemetry.addData("flywheel speed (ticks per second)", hardware.flywheel.getVelocity());
        telemetry.addData("flywheel2 power", hardware.flywheel2.getPower());
        telemetry.addData("flywheel up to speed ", shooter.flywheelUpToSpeed());
        telemetry.addData("velocity compensation", shooter.velComp);
        telemetry.update();
    }

    private void driveUpdate() {
        drive.headingLock = gamepad1.cross;

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
                    && shooter.flywheelUpToSpeed()) {
                shooter.setState(Shooter.State.LAUNCH);
            } else if (gamepad2.share) {
                shooter.setState(Shooter.State.RESET);
            }
        }

        if (gamepad2.touchpad && !lastGamepad2.touchpad && shooter.velComp) {
            shooter.velComp = false;
        } else if (gamepad2.touchpad && !lastGamepad2.touchpad && !shooter.velComp) {
            shooter.velComp = true;
        }

        if (gamepad2.options && !lastGamepad2.options && shooter.runTurret) {
            shooter.runTurret = false;
            gamepad2.rumble(0.5, 0.5, 500);
        } else if (gamepad2.options && !lastGamepad2.options && !shooter.runTurret) {
            shooter.runTurret = true;
            gamepad2.rumble(0.5, 0.5, 500);
        }

        gamepad2.setLedColor(shooter.velComp ? 0 : 1, shooter.velComp ? 1 : 0, 0, -1);

        shooter.turretOffset(gamepad2.left_stick_x * ShooterConstants.TURRET_TRIM_SPEED);

        ShooterConstants.flywheelOffset(((gamepad2.dpad_up ? 1 : 0) + (gamepad2.dpad_down ? -1 : 0))
                * ShooterConstants.FLYWHEEL_TRIM_SPEED);
        ShooterConstants.hoodOffset(gamepad2.right_stick_y * ShooterConstants.HOOD_TRIM_SPEED);

        prevShooterState = shooter.getState();
    }
}
