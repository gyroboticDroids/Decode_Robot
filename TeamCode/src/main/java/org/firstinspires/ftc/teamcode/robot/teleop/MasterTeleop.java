package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.subassamblies.Drive;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Intake;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Shooter;

import java.util.List;

public class MasterTeleop extends OpMode {
    private Hardware hardware;

    private Drive drive;
    private Intake intake;
    private Shooter shooter;

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
    }

    @Override
    public void start() {
        intake.setState(Intake.State.INTAKE);
        shooter.setState(Shooter.State.SLEEP);
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
    }

    private void driveUpdate(){
        if(gamepad1.a)
            drive.headingLock = true;
        else if (gamepad1.b)
            drive.headingLock = false;

    }

    private void intakeUpdate(){

    }

    private void shooterUpdate(){

    }
}
