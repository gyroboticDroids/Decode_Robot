package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    //vision
    public Limelight3A limelight;

    public Hardware(HardwareMap hardwareMap)
    {
        hmap = hardwareMap;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
    }

    public void configureTeleop(){
        leftFront = hmap.get(DcMotor.class, "lf");
        leftRear = hmap.get(DcMotor.class, "lr");
        rightFront = hmap.get(DcMotor.class, "rf");
        rightRear = hmap.get(DcMotor.class, "rr");

        poseTracker = new PoseTracker();
    }
}
