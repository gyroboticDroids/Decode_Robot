package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Intake;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Shooter;

import java.util.EnumMap;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "close auto", group = "auto", preselectTeleOp = "Master Tele-op")
public class CloseAuto extends OpMode {
    private enum PoseName{
        START, SCORE_PRELOAD, SPIKE_MARK_1, SCORE, GATE, SPIKE_MARK_2, SPIKE_MARK_3
    }

    EnumMap<PoseName, Pose> poses = new EnumMap<>(PoseName.class);

    private Path scorePreload, collectBalls1, scoreBalls1, openGate, collectBalls2, scoreBalls2, collectBalls3, scoreBalls3;

    private Follower follower;
    private Timer pathTimer;

    private Hardware hardware;
    private Intake intake;
    private Shooter shooter;

    private boolean ons = false;
    private boolean robotAtEnd = false;

    private int pathState = -1;
    private boolean allianceColorRed = true;


    @Override
    public void init() {
        setUpPoses();

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(poses.get(PoseName.START));

        hardware = new Hardware(hardwareMap);
        hardware.setPoseTrackerInAuto(follower.poseTracker);

        intake = new Intake(hardware);
        shooter = new Shooter(hardware);

        intake.setState(Intake.State.INTAKE_SLEEP);
        shooter.setState(Shooter.State.OFF);
    }

    @Override
    public void init_loop() {
        if (gamepad1.crossWasPressed())
            allianceColorRed = !allianceColorRed;

        telemetry.addLine("alliance " + (allianceColorRed ? "RED" : "BLUE"));
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);

        if (!allianceColorRed) {
            mirrorPoses();
        }

        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("robot at end", robotAtEnd);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        robotAtEnd = follower.getCurrentTValue() > 0.98;

        switch (pathState) {
            case 0:
                intake.setState(Intake.State.INTAKE);
                shooter.setState(Shooter.State.READY);

                follower.followPath(scorePreload);

                setPathState(pathState + 1);
                break;

            case 1:
                if (!shooter.isBusy() && robotAtEnd) {
                    shooter.setState(Shooter.State.LAUNCH);

                    setPathState(pathState + 1);
                }
                break;

            case 2:
                if (!shooter.isBusy()) {
                    shooter.setState(Shooter.State.READY);
                    follower.followPath(collectBalls1);

                    setPathState(pathState + 1);
                }
                break;

            case 3:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(scoreBalls1);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 4:
                if (robotAtEnd) {
                    shooter.setState(Shooter.State.LAUNCH);

                    setPathState(pathState + 1);
                }
                break;

            case 5:
                if (!shooter.isBusy()) {
                    intake.setState(Intake.State.INTAKE_UP);
                    shooter.setState(Shooter.State.READY);
                    follower.followPath(openGate);

                    setPathState(pathState + 1);
                }
                break;

            case 6:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(collectBalls2);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 7:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(scoreBalls2);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 8:
                if (robotAtEnd) {
                    shooter.setState(Shooter.State.LAUNCH);

                    setPathState(pathState + 1);
                }
                break;

            case 9:
                if (!shooter.isBusy()) {
                    shooter.setState(Shooter.State.READY);
                    follower.followPath(collectBalls3);

                    setPathState(pathState + 1);
                }
                break;

            case 10:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        follower.followPath(scoreBalls3);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 11:
                if (robotAtEnd) {
                    shooter.setState(Shooter.State.LAUNCH);

                    setPathState(pathState + 1);
                }
                break;

            case 12:
                if (!shooter.isBusy()) {
                    shooter.setState(Shooter.State.READY);
                    follower.followPath(collectBalls2);

                    setPathState(-1);
                }
                break;
        }
    }

    private void setUpPoses() {
        poses.put(PoseName.START, new Pose(0, 0, Math.toRadians(270)));
        poses.put(PoseName.SCORE_PRELOAD, new Pose(0, 0, Math.toRadians(270)));
        poses.put(PoseName.SPIKE_MARK_1, new Pose(0, 0, Math.toRadians(270)));
        poses.put(PoseName.SCORE, new Pose(0, 0, Math.toRadians(270)));
        poses.put(PoseName.GATE, new Pose(0, 0, Math.toRadians(270)));
        poses.put(PoseName.SPIKE_MARK_2, new Pose(0, 0, Math.toRadians(270)));
        poses.put(PoseName.SPIKE_MARK_3, new Pose(0, 0, Math.toRadians(270)));
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(poses.get(PoseName.START), poses.get(PoseName.SCORE_PRELOAD)));
        scorePreload.setConstantHeadingInterpolation(270);

        collectBalls1 = new Path(new BezierLine(poses.get(PoseName.SCORE_PRELOAD), poses.get(PoseName.SPIKE_MARK_1)));
        collectBalls1.setConstantHeadingInterpolation(270);

        scoreBalls1 = new Path(new BezierLine(poses.get(PoseName.SPIKE_MARK_1), poses.get(PoseName.SCORE)));
        scoreBalls1.setConstantHeadingInterpolation(270);

        openGate = new Path(new BezierLine(poses.get(PoseName.SCORE), poses.get(PoseName.GATE)));
        openGate.setConstantHeadingInterpolation(270);

        collectBalls2 = new Path(new BezierLine(poses.get(PoseName.GATE), poses.get(PoseName.SPIKE_MARK_2)));
        collectBalls2.setConstantHeadingInterpolation(270);

        scoreBalls2 = new Path(new BezierLine(poses.get(PoseName.SPIKE_MARK_2), poses.get(PoseName.SCORE)));
        scoreBalls2.setConstantHeadingInterpolation(270);

        collectBalls3 = new Path(new BezierLine(poses.get(PoseName.SCORE), poses.get(PoseName.SPIKE_MARK_3)));
        collectBalls3.setConstantHeadingInterpolation(270);

        scoreBalls3 = new Path(new BezierLine(poses.get(PoseName.SPIKE_MARK_3), poses.get(PoseName.SCORE)));
        scoreBalls3.setConstantHeadingInterpolation(270);
    }

    private void mirrorPoses() {
        for (Pose pose : poses.values()) {
            pose.mirror();
        }
    }

    public void setPathState(int p) {
        pathState = p;
        ons = true;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        TransferConstants.endPose = follower.getPose();
        TransferConstants.isAllianceColorRed = allianceColorRed;
        TransferConstants.endTurretPos = hardware.turret.getCurrentPosition();
    }
}
