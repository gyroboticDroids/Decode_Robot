package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Hardware {
    HardwareMap hardwareMap;

    //drive
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;
    public DcMotorEx rightRear;

    public Servo parkLeft;
    public Servo parkRight;

    public PoseTracker poseTracker;

    //intake
    public DcMotorEx intake;

    public Servo intakePivotRight;

    //shooter
    public DcMotorEx flywheel;
    public DcMotorEx flywheel2;
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
        this.hardwareMap = hardwareMap;

        //drive
        parkLeft = this.hardwareMap.get(Servo.class, "parkLeft");
        parkRight = this.hardwareMap.get(Servo.class, "parkRight");

        //intake
        intake = this.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        intakePivotRight = this.hardwareMap.get(Servo.class, "intakeRight");

        //shooter
        flywheel = this.hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel2 = this.hardwareMap.get(DcMotorEx.class, "shooter2");
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        turret = this.hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        hood = this.hardwareMap.get(Servo.class, "hood");
        launcher = this.hardwareMap.get(Servo.class, "launcher");
        door = this.hardwareMap.get(Servo.class, "door");

        ball1 = this.hardwareMap.get(RevColorSensorV3.class, "ball1");
        ball2 = this.hardwareMap.get(RevColorSensorV3.class, "ball2");
        ball3 = this.hardwareMap.get(RevColorSensorV3.class, "ball3");

        //vision
        limelight = this.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
    }

    public void configureTeleop() {
        //drive
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConstants);
        poseTracker = new PoseTracker(localizer);
    }

    public void setPoseTrackerInAuto(PoseTracker poseTracker) {
        this.poseTracker = poseTracker;
    }
}
