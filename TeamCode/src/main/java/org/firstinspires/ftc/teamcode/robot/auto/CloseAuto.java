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

@Autonomous(name = "close auto", group = "auto", preselectTeleOp = "Master Tele-op")
public class CloseAuto extends OpMode {

    Pose[] poses = new Pose[8];

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
        poses[0] = new Pose(126, 122.74, Math.toRadians(270));//START
        poses[1] = new Pose(112, 112, Math.toRadians(320));//SCORE_PRELOAD
        poses[2] = new Pose(121, 104);//SPIKE_MARK_1_CONTROL
        poses[3] = new Pose(120, 84 + 8, Math.toRadians(270));//SPIKE_MARK_1
        poses[4] = new Pose(112, 112, Math.toRadians(320));//SCORE
        poses[5] = new Pose(120, 60 + 8, Math.toRadians(270));//SPIKE_MARK_2
        poses[6] = new Pose(130, 70, Math.toRadians(270));//GATE
        poses[7] = new Pose(120, 36 + 8, Math.toRadians(270));//SPIKE_MARK_3
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(poses[0], poses[1]));
        scorePreload.setLinearHeadingInterpolation(poses[0].getHeading(), poses[1].getHeading());

        collectBalls1 = new Path(new BezierCurve(poses[1], poses[2], poses[3]));
        collectBalls1.setLinearHeadingInterpolation(poses[1].getHeading(), poses[3].getHeading(), 0.5);

        scoreBalls1 = new Path(new BezierLine(poses[3], poses[4]));
        scoreBalls1.setLinearHeadingInterpolation(poses[3].getHeading(), poses[4].getHeading());

        collectBalls2 = new Path(new BezierLine(poses[4], poses[5]));
        collectBalls2.setLinearHeadingInterpolation(poses[4].getHeading(), poses[5].getHeading(), 0.7);

        openGate = new Path(new BezierLine(poses[5], poses[6]));
        openGate.setLinearHeadingInterpolation(poses[5].getHeading(), poses[6].getHeading());

        scoreBalls2 = new Path(new BezierLine(poses[5], poses[4]));
        scoreBalls2.setLinearHeadingInterpolation(poses[5].getHeading(), poses[4].getHeading());

        collectBalls3 = new Path(new BezierLine(poses[4], poses[7]));
        collectBalls3.setLinearHeadingInterpolation(poses[4].getHeading(), poses[7].getHeading(), 0.7);

        scoreBalls3 = new Path(new BezierLine(poses[7], poses[4]));
        scoreBalls3.setLinearHeadingInterpolation(poses[7].getHeading(), poses[4].getHeading());
    }

    @Override
    public void init() {
        setUpPoses();

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.7);

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
        }

        if (shooter.getState() != Shooter.State.RESET) {
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

        follower.setStartingPose(poses[0]);
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
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.setState(Intake.State.INTAKE);
                        shooter.setState(Shooter.State.LAUNCH);

                        setPathState(pathState + 1);
                    }
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
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.setState(Intake.State.INTAKE);
                        shooter.setState(Shooter.State.LAUNCH);

                        setPathState(pathState + 1);
                    }
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

                    if (pathTimer.getElapsedTimeSeconds() > 1 || shooter.areBallsCollected()) {
                        intake.setState(Intake.State.INTAKE_UP);
                        follower.followPath(scoreBalls2);
                        setPathState(pathState + 1);
                    }
                }
                break;

            case 8:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.setState(Intake.State.INTAKE);
                        shooter.setState(Shooter.State.LAUNCH);

                        setPathState(pathState + 1);
                    }
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
                if (robotAtEnd || !ons) {
                    if (ons) {
                        pathTimer.resetTimer();
                        ons = false;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        intake.setState(Intake.State.INTAKE);
                        shooter.setState(Shooter.State.LAUNCH);

                        setPathState(pathState + 1);
                    }
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
        for(int i = 0; i < poses.length; i++) {
            poses[i] = poses[i].mirror();
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
        TransferConstants.endTurretPos = hardware.turret.getCurrentPosition();
    }
}
