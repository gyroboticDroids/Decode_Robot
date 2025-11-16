package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Intake;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Shooter;

import java.util.EnumMap;

@Autonomous(name = "close auto", group = "auto", preselectTeleOp = "Master Tele-op")
public class CloseAuto extends OpMode {
    private enum PoseName {
        START, SCORE_PRELOAD, SPIKE_MARK_1, SCORE, GATE, SPIKE_MARK_2, SPIKE_MARK_2_CONTROL, SPIKE_MARK_3, SPIKE_MARK_3_CONTROL
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

    private void setUpPoses() {
        poses.put(PoseName.START, new Pose(126, 122.74, Math.toRadians(270)));
        poses.put(PoseName.SCORE_PRELOAD, new Pose(85, 84, Math.toRadians(0)));
        poses.put(PoseName.SPIKE_MARK_1, new Pose(120, 84, Math.toRadians(0)));
        poses.put(PoseName.SCORE, new Pose(85, 84, Math.toRadians(0)));
        poses.put(PoseName.SPIKE_MARK_2, new Pose(120, 60, Math.toRadians(0)));
        poses.put(PoseName.SPIKE_MARK_2_CONTROL, new Pose(90, 57));
        poses.put(PoseName.GATE, new Pose(130, 70, Math.toRadians(270)));
        poses.put(PoseName.SPIKE_MARK_3, new Pose(120, 36, Math.toRadians(0)));
        poses.put(PoseName.SPIKE_MARK_3_CONTROL, new Pose(90, 32));
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(poses.get(PoseName.START), poses.get(PoseName.SCORE_PRELOAD)));
        scorePreload.setLinearHeadingInterpolation(poses.get(PoseName.START).getHeading(), poses.get(PoseName.SCORE_PRELOAD).getHeading());

        collectBalls1 = new Path(new BezierLine(poses.get(PoseName.SCORE_PRELOAD), poses.get(PoseName.SPIKE_MARK_1)));
        collectBalls1.setConstantHeadingInterpolation(poses.get(PoseName.SPIKE_MARK_1).getHeading());

        scoreBalls1 = new Path(new BezierLine(poses.get(PoseName.SPIKE_MARK_1), poses.get(PoseName.SCORE)));
        scoreBalls1.setConstantHeadingInterpolation(poses.get(PoseName.SCORE).getHeading());

        collectBalls2 = new Path(new BezierCurve(poses.get(PoseName.SCORE), poses.get(PoseName.SPIKE_MARK_2_CONTROL),
                poses.get(PoseName.SPIKE_MARK_2)));
        collectBalls2.setConstantHeadingInterpolation(poses.get(PoseName.SPIKE_MARK_2).getHeading());

        openGate = new Path(new BezierLine(poses.get(PoseName.SPIKE_MARK_2), poses.get(PoseName.GATE)));
        openGate.setLinearHeadingInterpolation(poses.get(PoseName.SPIKE_MARK_2).getHeading(), poses.get(PoseName.GATE).getHeading(), 0.7);

        scoreBalls2 = new Path(new BezierLine(poses.get(PoseName.SPIKE_MARK_2), poses.get(PoseName.SCORE)));
        scoreBalls2.setLinearHeadingInterpolation(poses.get(PoseName.SPIKE_MARK_2).getHeading(), poses.get(PoseName.SCORE).getHeading());

        collectBalls3 = new Path(new BezierCurve(poses.get(PoseName.SCORE), poses.get(PoseName.SPIKE_MARK_3_CONTROL),
                poses.get(PoseName.SPIKE_MARK_3)));
        collectBalls3.setConstantHeadingInterpolation(poses.get(PoseName.SPIKE_MARK_3).getHeading());

        scoreBalls3 = new Path(new BezierLine(poses.get(PoseName.SPIKE_MARK_3), poses.get(PoseName.SCORE)));
        scoreBalls3.setConstantHeadingInterpolation(poses.get(PoseName.SCORE).getHeading());
    }

    @Override
    public void init() {
        setUpPoses();

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        hardware = new Hardware(hardwareMap);
        hardware.setPoseTrackerInAuto(follower.poseTracker);

        intake = new Intake(hardware);
        shooter = new Shooter(hardware);

        intake.setState(Intake.State.INTAKE_SLEEP);
        shooter.setState(Shooter.State.RESET);
    }

    @Override
    public void init_loop() {
        if (gamepad1.crossWasReleased())
            allianceColorRed = !allianceColorRed;

        if (shooter.getState() == Shooter.State.RESET && !shooter.isBusy()) {
            shooter.setState(Shooter.State.OFF);
            telemetry.addLine("READY!");
        }

        intake.update();
        shooter.update();

        telemetry.addLine("alliance " + (allianceColorRed ? "RED" : "BLUE"));
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);

        if (!allianceColorRed) {
            mirrorPoses();
        }

        TransferConstants.isAllianceColorRed = allianceColorRed;

        follower.setStartingPose(poses.get(PoseName.START));
        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        intake.update();
        shooter.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("robot at end", robotAtEnd);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        robotAtEnd = follower.getCurrentTValue() > 0.99;

        switch (pathState) {
            case 0:
                intake.setState(Intake.State.INTAKE_UP);
                shooter.setState(Shooter.State.READY);

                follower.followPath(scorePreload);

                setPathState(pathState + 1);
                break;

            case 1:
                if (!shooter.isBusy() && robotAtEnd) {
                    intake.setState(Intake.State.INTAKE);
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

                    if (pathTimer.getElapsedTimeSeconds() > 1 || shooter.areBallsCollected()) {
                        intake.setState(Intake.State.INTAKE_UP);
                        follower.followPath(scoreBalls1);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 4:
                if (robotAtEnd) {
                    intake.setState(Intake.State.INTAKE);
                    shooter.setState(Shooter.State.LAUNCH);

                    setPathState(pathState + 1);
                }
                break;

            case 5:
                if (!shooter.isBusy()) {
                    shooter.setState(Shooter.State.READY);
                    follower.followPath(collectBalls2);
                    setPathState(7);
                }
                break;

            case 6:
                if (robotAtEnd) {
                    intake.setState(Intake.State.INTAKE_UP);
                    follower.followPath(openGate);

                    setPathState(pathState + 1);
                }
                break;

            case 7:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        intake.setState(Intake.State.INTAKE_UP);
                        follower.followPath(scoreBalls2);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 8:
                if (robotAtEnd) {
                    intake.setState(Intake.State.INTAKE);
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

                    if (pathTimer.getElapsedTimeSeconds() > 1 || shooter.areBallsCollected()) {
                        intake.setState(Intake.State.INTAKE_UP);
                        follower.followPath(scoreBalls3);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 11:
                if (robotAtEnd) {
                    intake.setState(Intake.State.INTAKE);
                    shooter.setState(Shooter.State.LAUNCH);

                    setPathState(pathState + 1);
                }
                break;

            case 12:
                if (!shooter.isBusy()) {
                    intake.setState(Intake.State.INTAKE_UP);
                    shooter.setState(Shooter.State.OFF);
                    follower.followPath(collectBalls1);

                    setPathState(-1);
                }
                break;
        }
    }

    private void mirrorPoses() {
        poses.put(PoseName.START, poses.get(PoseName.START).mirror());
        poses.put(PoseName.SCORE_PRELOAD, poses.get(PoseName.SCORE_PRELOAD).mirror());
        poses.put(PoseName.SPIKE_MARK_1, poses.get(PoseName.SPIKE_MARK_1).mirror());
        poses.put(PoseName.SCORE, poses.get(PoseName.SCORE).mirror());
        poses.put(PoseName.SPIKE_MARK_2, poses.get(PoseName.SPIKE_MARK_2).mirror());
        poses.put(PoseName.SPIKE_MARK_2_CONTROL, poses.get(PoseName.SPIKE_MARK_2_CONTROL).mirror());
        poses.put(PoseName.GATE, poses.get(PoseName.GATE).mirror());
        poses.put(PoseName.SPIKE_MARK_3, poses.get(PoseName.SPIKE_MARK_3).mirror());
        poses.put(PoseName.SPIKE_MARK_3_CONTROL, poses.get(PoseName.SPIKE_MARK_3_CONTROL).mirror());
    }

    public void setPathState(int p) {
        pathState = p;
        ons = true;
        pathTimer.resetTimer();
    }

    @Override
    public void stop() {
        TransferConstants.endPose = follower.getPose();
        TransferConstants.endTurretPos = hardware.turret.getCurrentPosition();
    }
}
