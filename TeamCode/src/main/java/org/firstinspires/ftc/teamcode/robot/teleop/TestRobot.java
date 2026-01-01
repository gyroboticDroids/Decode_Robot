package org.firstinspires.ftc.teamcode.robot.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Vision;
@Configurable
@TeleOp(name = "test robot", group = "testing")
public class TestRobot extends OpMode {

    private Hardware hardware;
    private Vision vision;
    private int state = 0;

    private double parkLeftPos = 0.5;
    private double parkRightPos = 0.5;

    private double leftPivot = 0.5;
    private double rightPivot = 0.5;

    private double flywheelSpeed = 0;
    private double launcher = 0.5;
    private double hood = 0.5;
    private double door = 0.5;

    private Telemetry panelsTelemetry;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        hardware = new Hardware(hardwareMap);
        vision = new Vision(hardware);
        hardware.configureTeleop();

        hardware.parkLeft.setPosition(0.5);
        hardware.parkRight.setPosition(0.5);

        hardware.intakePivotRight.setPosition(0.5);

        hardware.door.setPosition(0.5);
        hardware.hood.setPosition(0.5);
        hardware.launcher.setPosition(0.5);
    }

    @Override
    public void loop() {
        hardware.flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstants.FLYWHEEL_PIDF);
        hardware.flywheel2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstants.FLYWHEEL_PIDF);

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

                hardware.intakePivotRight.setPosition(rightPivot);

                telemetry.addData("intake (g1 left stick y)", hardware.intake.getPower());

                telemetry.addData("right pivot (g2 right stick y)", hardware.intakePivotRight.getPosition());
                break;

            case 2:
                telemetry.addLine("----------Shooter---------");

                flywheelSpeed += gamepad1.left_stick_y * 1;

                if (gamepad1.cross)
                    flywheelSpeed = 0;

                hardware.flywheel.setVelocity(flywheelSpeed);
                hardware.flywheel2.setVelocity(flywheelSpeed);

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

                telemetry.addData("flywheel setpoint", flywheelSpeed);
                telemetry.addData("flywheel (g1 left stick y, cross to reset)", hardware.flywheel.getVelocity());
                telemetry.addData("flywheel2 power", hardware.flywheel2.getPower());
                telemetry.addData("turret degrees (g1 dpad left right)", hardware.turret.getCurrentPosition()
                        / ShooterConstants.TURRET_TICKS_PER_DEGREE);
                telemetry.addData("turret ticks", hardware.turret.getCurrentPosition());
                panelsTelemetry.addData("flywheel setpoint", flywheelSpeed);
                panelsTelemetry.addData("flywheel speed", hardware.flywheel.getVelocity());

                telemetry.addData("launcher (g1 right stick y)", hardware.launcher.getPosition());
                telemetry.addData("hood (g2 left stick y)", hardware.hood.getPosition());
                telemetry.addData("door (g2 right stick y)", hardware.door.getPosition());

                telemetry.addData("color 1 inches", hardware.ball1.getDistance(DistanceUnit.INCH));
                telemetry.addData("color 2 inches", hardware.ball2.getDistance(DistanceUnit.INCH));
                telemetry.addData("color 3 inches", hardware.ball3.getDistance(DistanceUnit.INCH));
                break;
        }

        telemetry.addData("robot pos from camera", vision.getRobotPosFromTarget());

        telemetry.update();
        panelsTelemetry.update();
    }
}
