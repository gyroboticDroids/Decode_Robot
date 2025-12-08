package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

@Autonomous(name = "run auto", group = "close auto", preselectTeleOp = "Master Tele-op")
public class RunAuto extends OpMode {
    private final static double MAX_POWER = 1;
    private final static double SLOW_POWER = 0.5;

    private List<Integer> routine;
    private int currentRoutineIndex = 0;
    int currentAction = 0;
    private int nextPath = -1;

    private Pose startClose = new Pose(126, 122.74, Math.toRadians(270)),
            startFar = new Pose(126, 122.74, Math.toRadians(90)),
            scoreClose = new Pose(120, 120, Math.toRadians(0)),
            scoreMiddle = new Pose(96, 96, Math.toRadians(0)),
            scoreFar = new Pose(84, 12, Math.toRadians(0)),
            balls1 = new Pose(120, 84, Math.toRadians(0)),
            balls2 = new Pose(120, 67, Math.toRadians(0)),
            balls3 = new Pose(120, 44, Math.toRadians(0)),
            gate = new Pose(128, 61, Math.toRadians(22)),
            hp = new Pose(130, 6, Math.toRadians(0)),
            endClose = new Pose(120, 84, Math.toRadians(0)),
            endFar = new Pose(120, 84, Math.toRadians(0));

    private Pose controlBalls1 = new Pose(95, 82),
            controlBalls2 = new Pose(96, 63),
            controlBalls3 = new Pose(120, 96),
            controlGate = new Pose(96, 72),
            controlHp = new Pose(130, 20);

    private Pose startPose;

    private List<PathChain> paths;

    private Follower follower;
    private Timer timer;

    private Hardware hardware;
    private Intake intake;
    private Shooter shooter;

    private boolean ons = false;
    private boolean isBusy = false;
    private boolean robotAtEnd = false;

    private int state = -1;
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
        paths = new ArrayList<>();
        loadRoutine();
    }

    private void loadRoutine() {
        File seqAutoFile = AppUtil.getInstance().getSettingsFile("Config.txt");
        String[] types = ReadWriteFile.readFile(seqAutoFile).trim().split(", ");

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
            telemetry.addData("routine", routine);
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

        intake.update(shooter.flywheelUpToSpeed());
        shooter.update();

        telemetry.addLine("alliance " + (allianceColorRed ? "RED" : "BLUE"));
        telemetry.update();
    }

    @Override
    public void start() {
        setState(0);

        if (!allianceColorRed) {
            mirrorPoses();
        }

        TransferConstants.isAllianceColorRed = allianceColorRed;

        if (routine.get(0) == 0) {
            startPose = startClose;
        } else {
            startPose = startFar;
        }

        follower.setStartingPose(startPose);
        buildPaths();
    }

    private void mirrorPoses() {
        startClose = startClose.mirror();
        startFar = startFar.mirror();
        scoreClose = scoreClose.mirror();
        scoreMiddle = scoreMiddle.mirror();
        scoreFar = scoreFar.mirror();
        balls1 = balls1.mirror();
        balls2 = balls2.mirror();
        balls3 = balls3.mirror();
        gate = gate.mirror();
        hp = hp.mirror();
        endClose = endClose.mirror();
        endFar = endFar.mirror();

        controlBalls1 = controlBalls1.mirror();
        controlBalls2 = controlBalls2.mirror();
        controlBalls3 = controlBalls3.mirror();
        controlGate = controlGate.mirror();
        controlHp = controlHp.mirror();
    }

    private void buildPaths() {
        Pose lastControlPoint = null;
        Pose lastPose = startPose;

        for (int i = 1; i < routine.size() - 1; i++) {
            switch (routine.get(i)) {
                case 10:
                    paths.add(follower.pathBuilder()
                            .addPath((lastControlPoint == null) ? new BezierCurve(lastPose, scoreClose) :
                                    new BezierCurve(lastPose, lastControlPoint, scoreClose))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), scoreClose.getHeading())
                            .setBrakingStrength(0.7)
                            .build());

                    lastControlPoint = null;
                    break;

                case 11:
                    paths.add(follower.pathBuilder()
                            .addPath((lastControlPoint == null) ? new BezierCurve(lastPose, scoreMiddle) :
                                    new BezierCurve(lastPose, lastControlPoint, scoreMiddle))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), scoreMiddle.getHeading())
                            .setBrakingStrength(0.7)
                            .build());

                    lastControlPoint = null;
                    break;

                case 12:
                    paths.add(follower.pathBuilder()
                            .addPath((lastControlPoint == null) ? new BezierCurve(lastPose, scoreFar) :
                                    new BezierCurve(lastPose, lastControlPoint, scoreFar))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), scoreFar.getHeading())
                            .setBrakingStrength(0.7)
                            .build());

                    lastControlPoint = null;
                    break;

                case 20:
                    paths.add(follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlBalls1, balls1))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), balls1.getHeading(), 0.5)
                            .setBrakingStrength(0.7)
                            .build());

                    lastControlPoint = controlBalls1;
                    break;

                case 21:
                    paths.add(follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlBalls2, balls2))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), balls2.getHeading(), 0.7)
                            .setBrakingStrength(0.8)
                            .build());

                    lastControlPoint = controlBalls2;
                    break;

                case 22:
                    paths.add(follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlBalls3, balls3))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), balls3.getHeading(), 0.7)
                            .setBrakingStrength(0.8)
                            .build());

                    lastControlPoint = controlBalls3;
                    break;

                case 30:
                    paths.add(follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlGate, gate))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), gate.getHeading(), 0.7)
                            .setBrakingStrength(0.6)
                            .build());

                    lastControlPoint = controlGate;
                    break;

                case 40:
                    paths.add(follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlHp, hp))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), hp.getHeading(), 0.7)
                            .setBrakingStrength(0.6)
                            .build());

                    lastControlPoint = controlHp;
                    break;
            }

            lastPose = paths.get(paths.size() - 1).endPose();
        }

        Pose endPose = paths.get(paths.size() - 1).endPose();

        if (endPose.equals(scoreFar)) {
            paths.add(follower.pathBuilder()
                    .addPath(new BezierLine(endPose, endFar))
                    .setLinearHeadingInterpolation(endPose.getHeading(), endFar.getHeading())
                    .setBrakingStrength(0.8)
                    .build());
        } else {
            paths.add(follower.pathBuilder()
                    .addPath(new BezierLine(endPose, endClose))
                    .setLinearHeadingInterpolation(endPose.getHeading(), endClose.getHeading())
                    .setBrakingStrength(0.8)
                    .build());
        }
    }

    @Override
    public void loop() {
        follower.update();
        intake.update(shooter.flywheelUpToSpeed());
        shooter.update();
        autonomousPathUpdate();

        telemetry.addData("path state", state);
        telemetry.addData("routine", routine);
        telemetry.addData("current step", currentRoutineIndex);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("robot at end", robotAtEnd);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        robotAtEnd = follower.getCurrentTValue() > 0.99;

        if (!isBusy) {
            nextAction();
        }

        if (currentAction >= 10 && currentAction <= 19){
            if (currentRoutineIndex > 1) {
                isBusy = score();
            } else {
                isBusy = scorePreload();
            }
        } else if (currentAction >= 20 && currentAction <= 29) {
            isBusy = spikeMarks();
        } else if (currentAction >= 30 && currentAction <= 39) {
            isBusy = gate();
        } else if (currentAction >= 40 && currentAction <= 49) {
            isBusy = hp();
        } else if (currentAction >= 50 && currentAction <= 59) {
            isBusy = pause();
        } else {
            park();
        }
    }

    private boolean scorePreload() {
        switch (state) {
            case 0:
                intake.setState(Intake.State.INTAKE_UP);
                shooter.setState(Shooter.State.READY);

                follower.followPath(getPath());

                setState(state + 1);
                break;

            case 1:
                if (!shooter.isBusy() && !ons) {
                    intake.setState(Intake.State.INTAKE_LAUNCH);
                    shooter.setState(Shooter.State.LAUNCH);
                    ons = true;
                }

                if (robotAtEnd && ons)
                    setState(state + 1);
                break;

            case 2:
                if (!shooter.isBusy()) {
                    intake.setState(Intake.State.INTAKE);
                    shooter.setState(Shooter.State.READY);
                    return false;
                }
                break;
        }

        return true;
    }

    private boolean score() {
        switch (state) {
            case 0:
                follower.followPath(getPath());
                setState(state + 1);
                break;

            case 1:
                if (robotAtEnd || ons) {
                    if (!ons) {
                        timer.resetTimer();
                        ons = true;
                    }

                    if (timer.getElapsedTimeSeconds() > 0.3) {
                        intake.setState(Intake.State.INTAKE_LAUNCH);
                        shooter.setState(Shooter.State.LAUNCH);

                        setState(state + 1);
                    }
                }
                break;

            case 2:
                if (!shooter.isBusy()) {
                    intake.setState(Intake.State.INTAKE);
                    shooter.setState(Shooter.State.READY);
                    return false;
                }
                break;
        }

        return true;
    }

    private boolean spikeMarks() {
        switch (state) {
            case 0:
                follower.followPath(getPath());
                setState(state + 1);
                break;

            case 1:
                if (robotAtEnd && ons || shooter.areBallsCollected()) {
                    setState(state + 1);
                }
                break;

            case 2:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 0.5) {
                    intake.setState(Intake.State.INTAKE_UP);
                    return false;
                }
                break;
        }

        return true;
    }

    private boolean gate() {
        switch (state) {
            case 0:
                follower.setMaxPower(SLOW_POWER);
                follower.followPath(getPath());
                setState(state + 1);
                break;

            case 1:
                if (robotAtEnd && ons || shooter.areBallsCollected()) {
                    follower.setMaxPower(MAX_POWER);
                    setState(state + 1);
                }
                break;

            case 2:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 2) {
                    intake.setState(Intake.State.INTAKE_UP);
                    return false;
                }
                break;
        }

        return true;
    }

    private boolean hp() {
        switch (state) {
            case 0:
                follower.setMaxPower(SLOW_POWER);
                follower.followPath(getPath());
                setState(state + 1);
                break;

            case 1:
                if (robotAtEnd && ons || shooter.areBallsCollected()) {
                    follower.setMaxPower(MAX_POWER);
                    setState(state + 1);
                }
                break;

            case 2:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 1) {
                    intake.setState(Intake.State.INTAKE_UP);
                    return false;
                }
                break;
        }

        return true;
    }

    private boolean pause() {
        return timer.getElapsedTimeSeconds() < 0.5;
    }

    private void park() {
        if (!ons) {
            intake.setState(Intake.State.INTAKE_SLEEP);
            shooter.setState(Shooter.State.OFF);
            follower.followPath(getPath());

            ons = true;
        }
    }

    private PathChain getPath() {
        nextPath++;
        return paths.get(nextPath);
    }

    private void setState(int p) {
        state = p;
        ons = false;
        timer.resetTimer();
    }

    private void nextAction() {
        currentRoutineIndex++;

        if (currentRoutineIndex <= routine.size() - 1) {
            currentAction = routine.get(currentRoutineIndex);
        } else {
            currentAction = -1;
        }

        isBusy = true;
        setState(0);
    }

    @Override
    public void stop() {
        TransferConstants.endPose = follower.getPose();
        TransferConstants.endTurretPos = hardware.turret.getCurrentPosition();
    }
}