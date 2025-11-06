package org.firstinspires.ftc.teamcode.robot.teleop;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;

public class TestRobot extends OpMode {
    private Hardware hardware;
    private int state = 0;

    private double parkLeftPos = 0.5;
    private double parkRightPos = 0.5;

    private double leftPivot = 0.5;
    private double rightPivot = 0.5;

    private double flywheelSpeed = 0;
    private double launcher = 0.5;
    private double hood = 0.5;
    private double door = 0.5;

    @Override
    public void init() {
        hardware = new Hardware(hardwareMap);

        hardware.parkLeft.setPosition(0.5);
        hardware.parkRight.setPosition(0.5);

        hardware.intakePivotLeft.setPosition(0.5);
        hardware.intakePivotRight.setPosition(0.5);

        hardware.door.setPosition(0.5);
        hardware.hood.setPosition(0.5);
        hardware.launcher.setPosition(0.5);
    }

    @Override
    public void loop() {
        if(gamepad1.optionsWasPressed()) {
            state++;
        }

        if (state > 2) {
            state = 0;
        }

        telemetry.addData("state (g1 options)", state);

        switch (state) {
            case 0:
                telemetry.addLine("----------Drive-----------");

                hardware.leftFront.setPower(gamepad1.left_stick_x);
                hardware.leftRear.setPower(gamepad1.left_stick_y);
                hardware.rightFront.setPower(gamepad1.right_stick_x);
                hardware.rightRear.setPower(gamepad1.right_stick_y);

                parkLeftPos += gamepad2.left_stick_y * 0.001;
                parkRightPos += gamepad2.right_stick_y * 0.001;

                parkLeftPos = MathFunctions.clamp(parkLeftPos, 0, 1);
                parkRightPos = MathFunctions.clamp(parkRightPos, 0, 1);

                hardware.parkLeft.setPosition(parkLeftPos);
                hardware.parkRight.setPosition(parkRightPos);

                telemetry.addData("left front (g1 left stick x)", hardware.leftFront.getPower());
                telemetry.addData("left rear (g1 left stick y)", hardware.leftRear.getPower());
                telemetry.addData("right front (g1 right stick x)", hardware.rightFront.getPower());
                telemetry.addData("right rear (g1 right stick y)", hardware.rightRear.getPower());

                telemetry.addData("left park (g2 left stick y)", hardware.parkLeft.getPosition());
                telemetry.addData("right park (g2 right stick y)", hardware.parkRight.getPosition());
                break;

            case 1:
                telemetry.addLine("----------Intake----------");

                hardware.intake.setPower(gamepad1.left_stick_y);

                leftPivot += gamepad2.left_stick_y * 0.001;
                rightPivot += gamepad2.right_stick_y * 0.001;

                leftPivot = MathFunctions.clamp(leftPivot, 0, 1);
                rightPivot = MathFunctions.clamp(rightPivot, 0, 1);

                hardware.intakePivotLeft.setPosition(leftPivot);
                hardware.intakePivotRight.setPosition(rightPivot);

                telemetry.addData("intake (g1 left stick y)", hardware.intake.getPower());

                telemetry.addData("left pivot (g2 left stick y)", hardware.intakePivotLeft.getPosition());
                telemetry.addData("right pivot (g2 right stick y)", hardware.intakePivotRight.getPosition());
                break;

            case 2:
                telemetry.addLine("----------Shooter---------");

                flywheelSpeed += gamepad1.left_stick_y * 1;

                hardware.flywheel.setVelocity(flywheelSpeed);
                hardware.turret.setPower(((gamepad1.dpad_left ? 1 : 0) - (gamepad1.dpad_right ? 1 : 0)) * 0.2);

                launcher += gamepad1.right_stick_y * 0.001;
                hood += gamepad2.left_stick_y * 0.001;
                door += gamepad2.right_stick_y * 0.001;

                launcher = MathFunctions.clamp(launcher, 0, 1);
                hood = MathFunctions.clamp(hood, 0, 1);
                door = MathFunctions.clamp(door, 0, 1);

                hardware.launcher.setPosition(launcher);
                hardware.hood.setPosition(hood);
                hardware.door.setPosition(door);

                telemetry.addData("flywheel (g1 left stick y)", hardware.flywheel.getVelocity());
                telemetry.addData("turret (g1 dpad left right)", hardware.turret.getCurrentPosition());

                telemetry.addData("launcher (g1 right stick y)", hardware.launcher.getPosition());
                telemetry.addData("hood (g2 left stick y)", hardware.hood.getPosition());
                telemetry.addData("door (g2 right stick y)", hardware.door.getPosition());
                break;
        }

        telemetry.update();
    }
}
