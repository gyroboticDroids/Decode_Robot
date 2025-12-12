package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Intake;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Shooter;

import java.util.EnumMap;

@Disabled
@Autonomous(name = "far auto", group = "auto", preselectTeleOp = "Master Tele-op")
public class FarAuto extends OpMode {
    private enum PoseName {
        START, SCORE_PRELOAD, SPIKE_MARK_3, SPIKE_MARK_3_CONTROL, SCORE, HUMAN_PLAYER_SPIKE_MARK, GATE_BALLS
    }

    EnumMap<PoseName, Pose> poses = new EnumMap<>(PoseName.class);

    private Path scorePreload, collectBalls1, scoreBalls1, collectBalls2, scoreBalls2, collectBalls3, scoreBalls3;

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
        poses.put(PoseName.START, new Pose(54.06, 4.58, Math.toRadians(90)));
        poses.put(PoseName.SCORE_PRELOAD, new Pose(64, 22, Math.toRadians(90)));
        poses.put(PoseName.SPIKE_MARK_3, new Pose(24, 36, Math.toRadians(180)));
        poses.put(PoseName.SPIKE_MARK_3_CONTROL, new Pose(80, 36));
        poses.put(PoseName.SCORE, new Pose(64, 22, Math.toRadians(180)));
        poses.put(PoseName.HUMAN_PLAYER_SPIKE_MARK, new Pose(12, 8, Math.toRadians(180)));
        poses.put(PoseName.GATE_BALLS, new Pose(12, 10, Math.toRadians(180)));
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(poses.get(PoseName.START), poses.get(PoseName.SCORE_PRELOAD)));
        scorePreload.setConstantHeadingInterpolation(poses.get(PoseName.SCORE_PRELOAD).getHeading());

        collectBalls1 = new Path(new BezierCurve(poses.get(PoseName.SCORE_PRELOAD), poses.get(PoseName.SPIKE_MARK_3_CONTROL),
                poses.get(PoseName.SPIKE_MARK_3)));
        collectBalls1.setLinearHeadingInterpolation(poses.get(PoseName.SCORE_PRELOAD).getHeading(),
                poses.get(PoseName.SPIKE_MARK_3).getHeading(), 0.7);

        scoreBalls1 = new Path(new BezierLine(poses.get(PoseName.SPIKE_MARK_3), poses.get(PoseName.SCORE)));
        scoreBalls1.setConstantHeadingInterpolation(poses.get(PoseName.SCORE).getHeading());

        collectBalls2 = new Path(new BezierLine(poses.get(PoseName.SCORE), poses.get(PoseName.HUMAN_PLAYER_SPIKE_MARK)));
        collectBalls2.setConstantHeadingInterpolation(poses.get(PoseName.HUMAN_PLAYER_SPIKE_MARK).getHeading());

        scoreBalls2 = new Path(new BezierLine(poses.get(PoseName.HUMAN_PLAYER_SPIKE_MARK), poses.get(PoseName.SCORE)));
        scoreBalls2.setConstantHeadingInterpolation(poses.get(PoseName.SCORE).getHeading());

        collectBalls3 = new Path(new BezierLine(poses.get(PoseName.SCORE), poses.get(PoseName.GATE_BALLS)));
        collectBalls3.setConstantHeadingInterpolation(poses.get(PoseName.GATE_BALLS).getHeading());

        scoreBalls3 = new Path(new BezierLine(poses.get(PoseName.GATE_BALLS), poses.get(PoseName.SCORE)));
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

        intake.setState(Intake.State.INTAKE_UP);
        shooter.setState(Shooter.State.OFF);
    }

    @Override
    public void init_loop() {
        if (gamepad1.crossWasReleased())
            allianceColorRed = !allianceColorRed;

        if (shooter.getState() != Shooter.State.RESET && !shooter.isBusy()) {
            shooter.setState(Shooter.State.OFF);
            telemetry.addLine("READY!");
        }

        telemetry.addLine("alliance " + (allianceColorRed ? "RED" : "BLUE"));
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);

        if (!allianceColorRed) {
            mirrorPoses();
        }

        follower.setStartingPose(poses.get(PoseName.START));
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

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
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
                        intake.setState(Intake.State.INTAKE_UP);
                        follower.followPath(scoreBalls2);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 7:
                if (robotAtEnd) {
                    intake.setState(Intake.State.INTAKE);
                    shooter.setState(Shooter.State.LAUNCH);

                    setPathState(pathState + 1);
                }
                break;

            case 8:
                if (!shooter.isBusy()) {
                    shooter.setState(Shooter.State.READY);
                    follower.followPath(collectBalls3);

                    setPathState(pathState + 1);
                }
                break;

            case 9:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        intake.setState(Intake.State.INTAKE_UP);
                        follower.followPath(scoreBalls3);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 10:
                if (robotAtEnd) {
                    intake.setState(Intake.State.INTAKE);
                    shooter.setState(Shooter.State.LAUNCH);

                    setPathState(pathState + 1);
                }
                break;

            case 11:
                if (!shooter.isBusy()) {
                    intake.setState(Intake.State.INTAKE_UP);
                    shooter.setState(Shooter.State.READY);
                    follower.followPath(collectBalls2);

                    setPathState(-1);
                }
                break;
        }
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
