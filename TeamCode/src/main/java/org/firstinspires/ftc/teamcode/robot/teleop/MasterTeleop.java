package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Drive;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Intake;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Shooter;

import java.util.List;

public class MasterTeleop extends OpMode {
    private Drive drive;
    private Intake intake;
    private Shooter shooter;

    private Intake.State prevIntakeState;
    private Shooter.State prevShooterState;

    //Bulk reading
    List<LynxModule> allHubs;

    @Override
    public void init() {
        Hardware hardware = new Hardware(hardwareMap);

        drive = new Drive(hardware, gamepad1);
        intake = new Intake(hardware);
        shooter = new Shooter(hardware);

        //Bulk reading
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void start() {
        intake.setState(Intake.State.INTAKE);
        shooter.setState(Shooter.State.SLEEP);

        prevIntakeState = intake.getState();
        prevShooterState = shooter.getState();
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
    }

    private void telemetryUpdate(){
        telemetry.addData("robot position", drive.getRobotPos());
        telemetry.addData("is heading lock", drive.headingLock);
        telemetry.update();
    }

    private void driveUpdate(){
        drive.headingLock = gamepad1.cross;
    }

    private void intakeUpdate(){
        if (!intake.isBusy()) {
            if (drive.isPark()) {
                intake.setState(Intake.State.INTAKE_UP);
            } else if((gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) && prevIntakeState != Intake.State.INTAKE) {
                intake.setState(Intake.State.INTAKE);
            } else if ((gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) && prevIntakeState == Intake.State.INTAKE){
                intake.setState(Intake.State.INTAKE_UP);
            } else if (gamepad2.cross) {
                intake.setState(Intake.State.CLEAR);
            } else if (gamepad2.crossWasReleased()) {
                intake.setState(Intake.State.INTAKE);
            }
        }

        prevIntakeState = intake.getState();
    }

    private void shooterUpdate(){
        if (!shooter.isBusy()) {
            if (drive.isPark()) {
                shooter.setState(Shooter.State.PARK);
            } else if (prevShooterState == Shooter.State.LAUNCH) {
                shooter.setState(Shooter.State.READY);
            } else if(gamepad1.right_bumper && gamepad2.right_bumper) {
                shooter.setState(Shooter.State.LAUNCH);
            } else if (gamepad2.triangle) {
                shooter.setState(Shooter.State.REJECT);
            } else if (gamepad2.share) {
                shooter.setState(Shooter.State.RESET);
            } else if (gamepad2.square) {
                shooter.setState(Shooter.State.SLEEP);
            }
        }

        shooter.turretOffset(gamepad2.left_stick_x * ShooterConstants.TURRET_TRIM_SPEED);

        ShooterConstants.flywheelOffset(((gamepad2.dpad_up ? 1 : 0) + (gamepad2.dpad_down ? -1 : 0))
                * ShooterConstants.FLYWHEEL_TRIM_SPEED);
        ShooterConstants.hoodOffset(gamepad2.right_stick_y * ShooterConstants.HOOD_TRIM_SPEED);

        prevShooterState = shooter.getState();
    }
}
