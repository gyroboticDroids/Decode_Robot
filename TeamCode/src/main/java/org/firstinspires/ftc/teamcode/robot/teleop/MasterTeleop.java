package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.subassamblies.Drive;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;

public class MasterTeleop extends OpMode {
    private Hardware hardware;

    private Drive drive;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);

        drive = new Drive(hardware, gamepad1);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        driveUpdate();

        drive.update();

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
}
