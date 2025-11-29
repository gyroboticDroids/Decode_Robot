package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Intake;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Shooter;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "run close auto", group = "close auto", preselectTeleOp = "Master Tele-op")
public class RunAuto extends OpMode {
    private final static double MAX_POWER = 1;
    private final static double SLOW_POWER = 0.5;

    private List<Integer> routine;
    private final int[] stateToRoutineConversion = {4, 4, 4, 6, 8, 10};
    private int currentRoutineIndex = 1;

    private Pose start = new Pose(126, 122.74, Math.toRadians(270)),
            score = new Pose(96, 96, Math.toRadians(330)),
            balls1 = new Pose(120, 84, Math.toRadians(0)),
            balls2 = new Pose(120, 67, Math.toRadians(315)),
            balls3 = new Pose(120, 44, Math.toRadians(270)),
            gate = new Pose(128, 61, Math.toRadians(22));

    private Pose controlBalls1 = new Pose(95, 82),
            controlBalls2 = new Pose(96, 63),
            controlBalls3 = new Pose(120, 96),
            controlGate = new Pose(96, 72);

    private Path scorePreload, collectBalls1, scoreBalls1, collectFromGate, scoreFromGate,
            collectBalls2, scoreBalls2, collectBalls3, scoreBalls3;

    private Follower follower;
    private Timer timer;

    private Hardware hardware;
    private Intake intake;
    private Shooter shooter;

    private boolean ons = false;
    private boolean robotAtEnd = false;

    private int pathState = -1;
    private boolean allianceColorRed = true;

    private boolean routineLoaded = false;

    @Override
    public void init() {
        timer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(MAX_POWER);

        hardware = new Hardware(hardwareMap);
        hardware.setPoseTrackerInAuto(follower.poseTracker);

        intake = new Intake(hardware);
        shooter = new Shooter(hardware);

        intake.setState(Intake.State.INTAKE_SLEEP);
        shooter.setState(Shooter.State.RESET);

        routine = new ArrayList<>();
        loadRoutine();
    }

    private void loadRoutine() {
        File closeAuto = AppUtil.getInstance().getSettingsFile("Config.txt");
        String[] types = ReadWriteFile.readFile(closeAuto).trim().split(", ");

        for (String type : types) {
            if (!type.isEmpty()) {
                routine.add(Integer.parseInt(type));
            }
        }

        if (!routine.isEmpty()) {
            routineLoaded = true;
        }
    }

    @Override
    public void init_loop() {
        if (routineLoaded) {
            telemetry.addLine("ROUTINE LOADED");
        } else {
            telemetry.addLine("ROUTINE CONTAINS 0 ACTIONS");
        }

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

        follower.setStartingPose(start);
        buildPaths();
    }

    private void mirrorPoses() {
        start = start.mirror();
        score = score.mirror();
        balls1 = balls1.mirror();
        balls2 = balls2.mirror();
        balls3 = balls3.mirror();
        gate = gate.mirror();

        controlBalls1 = controlBalls1.mirror();
        controlBalls2 = controlBalls2.mirror();
        controlBalls3 = controlBalls3.mirror();
        controlGate = controlGate.mirror();
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(start, score));
        scorePreload.setLinearHeadingInterpolation(start.getHeading(), score.getHeading());
        scorePreload.setBrakingStrength(0.8);

        collectBalls1 = new Path(new BezierCurve(score, controlBalls1, balls1));
        collectBalls1.setLinearHeadingInterpolation(score.getHeading(), balls1.getHeading(), 0.5);
        collectBalls1.setBrakingStrength(0.7);

        scoreBalls1 = new Path(new BezierLine(balls1, score));
        scoreBalls1.setLinearHeadingInterpolation(balls1.getHeading(), score.getHeading());
        scoreBalls1.setBrakingStrength(0.7);

        collectBalls2 = new Path(new BezierCurve(score, controlBalls2, balls2));
        collectBalls2.setLinearHeadingInterpolation(score.getHeading(), balls2.getHeading(), 0.7);
        collectBalls2.setBrakingStrength(0.8);

        scoreBalls2 = new Path(new BezierCurve(balls2, controlBalls2, score));
        scoreBalls2.setLinearHeadingInterpolation(balls2.getHeading(), score.getHeading());
        scoreBalls2.setBrakingStrength(0.8);

        collectBalls3 = new Path(new BezierCurve(score, controlBalls3, balls3));
        collectBalls3.setLinearHeadingInterpolation(score.getHeading(), balls3.getHeading(), 0.7);
        collectBalls3.setBrakingStrength(0.5);

        scoreBalls3 = new Path(new BezierLine(balls3, score));
        scoreBalls3.setLinearHeadingInterpolation(balls3.getHeading(), score.getHeading());
        scoreBalls3.setBrakingStrength(0.8);

        collectFromGate = new Path(new BezierCurve(score, controlGate, gate));
        collectFromGate.setLinearHeadingInterpolation(score.getHeading(), gate.getHeading(), 0.7);
        collectFromGate.setBrakingStrength(0.6);

        scoreFromGate = new Path(new BezierCurve(gate, controlGate, score));
        scoreFromGate.setLinearHeadingInterpolation(gate.getHeading(), score.getHeading());
        scoreFromGate.setBrakingStrength(0.8);
    }

    @Override
    public void loop() {
        follower.update();
        intake.update();
        shooter.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("routine", routine);
        telemetry.addData("current step", currentRoutineIndex);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("robot at end", robotAtEnd);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        robotAtEnd = follower.getCurrentTValue() >= 0.99;

        switch (pathState) {
            case 0:
                intake.setState(Intake.State.INTAKE_UP);
                shooter.setState(Shooter.State.READY);

                follower.followPath(scorePreload);

                setPathState(pathState + 1);
                break;

            case 2:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        timer.resetTimer();
                        ons = false;
                    }

                    if (timer.getElapsedTimeSeconds() > 0.3) {
                        intake.setState(Intake.State.INTAKE_LAUNCH);
                        shooter.setState(Shooter.State.LAUNCH);

                        setPathState(pathState + 1);
                    }
                }
                break;

            case 3:
                if (!shooter.isBusy()) {
                    shooter.setState(Shooter.State.READY);
                    intake.setState(Intake.State.INTAKE);
                    setPathState(-1);
                }
                break;

            case 4:
                if (robotAtEnd && !ons) {
                    setPathState(pathState + 1);
                }

                if (ons) {
                    follower.followPath(collectBalls1);
                    ons = false;
                }
                break;

            case 5:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 0.5) {
                    intake.setState(Intake.State.INTAKE_UP);

                    follower.followPath(scoreBalls1);
                    setPathState(1);
                }
                break;

            case 6:
                if (robotAtEnd && !ons) {
                    setPathState(pathState + 1);
                }

                if (ons) {
                    follower.setMaxPower(SLOW_POWER);
                    follower.followPath(collectBalls2);
                    ons = false;
                }
                break;

            case 7:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 0.5) {
                    intake.setState(Intake.State.INTAKE_UP);

                    follower.setMaxPower(MAX_POWER);
                    follower.followPath(scoreBalls2);
                    setPathState(1);
                }
                break;

            case 8:
                if (robotAtEnd && !ons) {
                    setPathState(pathState + 1);
                }

                if (ons) {
                    follower.setMaxPower(SLOW_POWER);
                    follower.followPath(collectBalls3);
                    ons = false;
                }
                break;

            case 9:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 0.5) {
                    intake.setState(Intake.State.INTAKE_UP);

                    follower.setMaxPower(MAX_POWER);
                    follower.followPath(scoreBalls3);
                    setPathState(1);
                }
                break;

            case 10:
                if (robotAtEnd && !ons) {
                    setPathState(pathState + 1);
                }

                if (ons) {
                    follower.followPath(collectFromGate);
                    ons = false;
                }
                break;

            case 11:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 2) {
                    intake.setState(Intake.State.INTAKE_UP);

                    follower.followPath(scoreFromGate);
                    setPathState(1);
                }
                break;

            case 12:
                if (timer.getElapsedTimeSeconds() > 1) {
                    setPathState(-1);
                }
                break;

            case -1:
                if (ons) {
                    intake.setState(Intake.State.INTAKE_SLEEP);
                    shooter.setState(Shooter.State.OFF);
                    follower.followPath(collectBalls1);

                    ons = false;
                }
                break;
        }
    }

    public void setPathState(int p) {
        if (p == -1) {
            if (currentRoutineIndex <= routine.size() - 1) {
                pathState = stateToRoutineConversion[routine.get(currentRoutineIndex)];
            } else {
                pathState = -1;
            }

            currentRoutineIndex += 2;
        } else {
            pathState = p;
        }

        ons = true;
        timer.resetTimer();
    }

    @Override
    public void stop() {
        TransferConstants.endPose = follower.getPose();
        TransferConstants.endTurretPos = hardware.turret.getCurrentPosition();
    }
}
