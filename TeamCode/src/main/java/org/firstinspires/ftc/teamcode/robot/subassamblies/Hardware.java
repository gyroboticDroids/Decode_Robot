package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;

public class Hardware {
    HardwareMap hmap;

    //drive
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

    public Servo parkLeft;
    public Servo parkRight;

    public PoseTracker poseTracker;

    //intake
    public DcMotor intake;

    public Servo intakePivotLeft;
    public Servo intakePivotRight;

    //shooter
    public DcMotorEx flywheel;
    public DcMotorEx turret;

    public Servo hood;
    public Servo launcher;
    public Servo door;

    public RevColorSensorV3 ball1;
    public RevColorSensorV3 ball2;
    public RevColorSensorV3 ball3;

    //vision
    public Limelight3A limelight;

    public Hardware(HardwareMap hardwareMap) {
        hmap = hardwareMap;

        //drive
        parkLeft = hmap.get(Servo.class, "parkLeft");
        parkRight = hmap.get(Servo.class, "parkRight");

        parkLeft.setPosition(DriveConstants.PARK_LEFT_UP_POS);
        parkRight.setPosition(DriveConstants.PARK_RIGHT_UP_POS);

        //intake
        intake = hmap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        intakePivotLeft = hmap.get(Servo.class, "intakeLeft");
        intakePivotRight = hmap.get(Servo.class, "intakeRight");

        //shooter
        flywheel = hmap.get(DcMotorEx.class, "shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ShooterConstants.FLYWHEEL_PIDF);

        turret = hmap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        hood = hmap.get(Servo.class, "hood");
        launcher = hmap.get(Servo.class, "launcher");
        door = hmap.get(Servo.class, "door");

        ball1 = hmap.get(RevColorSensorV3.class, "ball1");
        ball2 = hmap.get(RevColorSensorV3.class, "ball2");
        ball3 = hmap.get(RevColorSensorV3.class, "ball3");

        //vision
        limelight = hmap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
    }

    public void configureTeleop() {
        //drive
        leftFront = hmap.get(DcMotor.class, "lf");
        leftRear = hmap.get(DcMotor.class, "lr");
        rightFront = hmap.get(DcMotor.class, "rf");
        rightRear = hmap.get(DcMotor.class, "rr");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Localizer localizer = new PinpointLocalizer(hmap, Constants);
        poseTracker = new PoseTracker(localizer);
    }
}
