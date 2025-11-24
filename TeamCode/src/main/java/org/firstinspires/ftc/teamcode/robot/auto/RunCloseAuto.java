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

@Autonomous(name = "run close auto", group = "auto", preselectTeleOp = "Master Tele-op")
public class RunCloseAuto extends OpMode {
    private List<Integer> routine;
    private final int[] stateToRoutineConversion = {3, 5, 7, 9, 11};
    private int currentRoutineIndex = 0;

    private Pose start = new Pose(126, 122.74, Math.toRadians(270)),
            score = new Pose(112, 112, Math.toRadians(320)),
            balls1 = new Pose(120, 84 + 8, Math.toRadians(270)),
            balls2 = new Pose(120, 60 + 8, Math.toRadians(270)),
            balls3 = new Pose(120, 36 + 8, Math.toRadians(270)),
            gate = new Pose(120, 70, Math.toRadians(20));

    private Pose controlBalls1 = new Pose(121, 104),
            controlGate = new Pose(72, 72);

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
        follower.setMaxPower(0.7);

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
        File closeAuto = AppUtil.getInstance().getSettingsFile("CloseConfig.txt");
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
        controlGate = controlGate.mirror();
    }

    private void buildPaths() {
        scorePreload = new Path(new BezierLine(start, score));
        scorePreload.setLinearHeadingInterpolation(start.getHeading(), score.getHeading());

        collectBalls1 = new Path(new BezierCurve(score, controlBalls1, balls1));
        collectBalls1.setLinearHeadingInterpolation(score.getHeading(), balls1.getHeading(), 0.5);

        scoreBalls1 = new Path(new BezierLine(balls1, score));
        scoreBalls1.setLinearHeadingInterpolation(balls1.getHeading(), score.getHeading());

        collectBalls2 = new Path(new BezierLine(score, balls2));
        collectBalls2.setLinearHeadingInterpolation(score.getHeading(), balls2.getHeading(), 0.7);

        scoreBalls2 = new Path(new BezierLine(balls2, score));
        scoreBalls2.setLinearHeadingInterpolation(balls2.getHeading(), score.getHeading());

        collectBalls3 = new Path(new BezierLine(score, balls3));
        collectBalls3.setLinearHeadingInterpolation(score.getHeading(), balls3.getHeading(), 0.7);

        scoreBalls3 = new Path(new BezierLine(balls3, score));
        scoreBalls3.setLinearHeadingInterpolation(balls3.getHeading(), score.getHeading());

        collectFromGate = new Path(new BezierCurve(score, controlGate, gate));
        collectFromGate.setLinearHeadingInterpolation(score.getHeading(), gate.getHeading(), 0.7);

        scoreFromGate = new Path(new BezierCurve(gate, controlGate, score));
        scoreFromGate.setLinearHeadingInterpolation(gate.getHeading(), score.getHeading());
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

            case 1:
                if (robotAtEnd || !ons) {
                    if (ons) {
                        timer.resetTimer();
                        ons = false;
                    }

                    if (timer.getElapsedTimeSeconds() > 2) {
                        intake.setState(Intake.State.INTAKE_LAUNCH);
                        shooter.setState(Shooter.State.LAUNCH);

                        setPathState(pathState + 1);
                    }
                }
                break;

            case 2:
                if (!shooter.isBusy()) {
                    shooter.setState(Shooter.State.READY);
                    intake.setState(Intake.State.INTAKE);
                    setPathState(-1);
                }
                break;

            case 3:
                if (robotAtEnd && !ons) {
                    setPathState(pathState + 1);
                }

                if (ons) {
                    follower.followPath(collectBalls1);
                    ons = false;
                }
                break;

            case 4:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 1) {
                    intake.setState(Intake.State.INTAKE_UP);

                    follower.followPath(scoreBalls1);
                    setPathState(1);
                }
                break;

            case 5:
                if (robotAtEnd && !ons) {
                    setPathState(pathState + 1);
                }

                if (ons) {
                    follower.followPath(collectBalls2);
                    ons = false;
                }
                break;

            case 6:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 1) {
                    intake.setState(Intake.State.INTAKE_UP);

                    follower.followPath(scoreBalls2);
                    setPathState(1);
                }
                break;

            case 7:
                if (robotAtEnd && !ons) {
                    setPathState(pathState + 1);
                }

                if (ons) {
                    follower.followPath(collectBalls3);
                    ons = false;
                }
                break;

            case 8:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 1) {
                    intake.setState(Intake.State.INTAKE_UP);

                    follower.followPath(scoreBalls3);
                    setPathState(1);
                }
                break;

            case 9:
                if (robotAtEnd && !ons) {
                    setPathState(pathState + 1);
                }

                if (ons) {
                    follower.followPath(collectFromGate);
                    ons = false;
                }
                break;

            case 10:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 2) {
                    intake.setState(Intake.State.INTAKE_UP);

                    follower.followPath(scoreFromGate);
                    setPathState(1);
                }
                break;

            case 11:
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
            if (currentRoutineIndex < routine.size() - 1) {
                pathState = stateToRoutineConversion[routine.get(currentRoutineIndex)];
            } else {
                pathState = -1;
            }

            currentRoutineIndex++;
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
