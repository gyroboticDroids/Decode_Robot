package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
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
    private final static double SLOW_POWER = 0.8;

    private List<Integer> routine;
    private int currentRoutineIndex = 0;
    int currentAction = 0;
    private int nextPath = -1;

    private Pose startClose = new Pose(115.75, 124, Math.toRadians(45)),
            startFar = new Pose(77.3, 7.625, Math.toRadians(0)),
            scoreClose = new Pose(100, 100, Math.toRadians(0)),
            scoreMiddle = new Pose(86, 78, Math.toRadians(0)),
            scoreFar = new Pose(84, 19, Math.toRadians(0)),
            balls1 = new Pose(117, 84, Math.toRadians(0)),
            balls2 = new Pose(123, 57, Math.toRadians(0)),
            balls3 = new Pose(123, 33, Math.toRadians(0)),
            gate1 = new Pose(128, 57.5, Math.toRadians(37)),
            gate2 = new Pose(121, 63.25, Math.toRadians(0)),

            hp = new Pose(128, 8, Math.toRadians(0)),
            endClose = new Pose(120, 70, Math.toRadians(270)),
            endFar = new Pose(105, 33, Math.toRadians(0));

    private Pose controlBalls1 = new Pose(95, 84),
            controlBalls2 = new Pose(95, 57),
            controlBalls3 = new Pose(92, 33),
            controlGate = new Pose(100, 60),
            controlHp = new Pose(100, 10);

    private Pose startPose;

    private List<Path> paths;

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

    List<LynxModule> allHubs;

    @Override
    public void init() {
        timer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(MAX_POWER);

        hardware = new Hardware(hardwareMap);
        hardware.setPoseTrackerInAuto(follower.poseTracker);

        intake = new Intake(hardware);
        shooter = new Shooter(hardware);

        intake.setState(Intake.State.INTAKE_AUTO_READY);
        shooter.setState(Shooter.State.RESET);

        routine = new ArrayList<>();
        paths = new ArrayList<>();
        loadRoutine();

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
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
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

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

        intake.update();
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
        gate1 = gate1.mirror();
        gate2 = gate2.mirror();
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

        Path path;

        for (int i = 1; i < routine.size(); i++) {
            switch (routine.get(i)) {
                case 10:
                    path = new Path((lastControlPoint == null) ? new BezierCurve(lastPose, scoreClose) :
                            new BezierCurve(lastPose, lastControlPoint, scoreClose));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), scoreClose.getHeading(), 0.8);
                    path.setBrakingStrength(0.5);

                    paths.add(path);

                    lastControlPoint = null;
                    break;

                case 11:
                    path = new Path((lastControlPoint == null) ? new BezierCurve(lastPose, scoreMiddle) :
                            new BezierCurve(lastPose, lastControlPoint, scoreMiddle));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), scoreMiddle.getHeading(), 0.4);
                    path.setBrakingStrength(0.3);

                    paths.add(path);

                    lastControlPoint = null;
                    break;

                case 12:
                    path = new Path((lastControlPoint == null) ? new BezierCurve(lastPose, scoreFar) :
                            new BezierCurve(lastPose, lastControlPoint, scoreFar));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), scoreFar.getHeading(), 0.8);
                    path.setBrakingStrength(0.4);

                    paths.add(path);

                    lastControlPoint = null;
                    break;

                case 20:
                    path = new Path(new BezierCurve(lastPose, controlBalls1, balls1));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), balls1.getHeading(), 0.5);
                    path.setBrakingStrength(0.6);

                    paths.add(path);

                    lastControlPoint = controlBalls1;
                    break;

                case 21:
                    path = new Path(new BezierCurve(lastPose, controlBalls2, balls2));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), balls2.getHeading(), 0.7);
                    path.setBrakingStrength(0.7);

                    paths.add(path);

                    lastControlPoint = controlBalls2;
                    break;

                case 22:
                    path = new Path(new BezierCurve(lastPose, controlBalls3, balls3));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), balls3.getHeading(), 0.7);
                    path.setBrakingStrength(0.7);

                    paths.add(path);

                    lastControlPoint = controlBalls3;
                    break;

                case 30:
                    path = new Path(new BezierCurve(lastPose, controlGate, gate1));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), gate1.getHeading(), 0.7);
                    path.setBrakingStrength(0.3);

                    paths.add(path);

                    lastControlPoint = controlGate;
                    break;

                case 31:
                    path = new Path(new BezierCurve(lastPose, controlGate, gate1));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), gate1.getHeading(), 0.7);
                    path.setBrakingStrength(0.3);

                    paths.add(path);

                    path = new Path(new BezierLine(gate1, gate2));
                    path.setLinearHeadingInterpolation(gate1.getHeading(), gate2.getHeading(), 1);
                    path.setBrakingStrength(0.3);

                    paths.add(path);

                    lastControlPoint = controlGate;
                    break;

                case 40:
                    path = new Path(new BezierCurve(lastPose, controlHp, hp));
                    path.setLinearHeadingInterpolation(lastPose.getHeading(), hp.getHeading(), 0.7);
                    path.setBrakingStrength(0.6);

                    paths.add(path);

                    lastControlPoint = controlHp;
                    break;
            }

            lastPose = paths.get(paths.size() - 1).getPose(1);
        }

        if (lastPose.distanceFrom(scoreFar) < 1) {
            path = new Path(new BezierLine(lastPose, endFar));
            path.setLinearHeadingInterpolation(lastPose.getHeading(), endFar.getHeading());

        } else {
            path = new Path(new BezierLine(lastPose, endClose));
            path.setLinearHeadingInterpolation(lastPose.getHeading(), endClose.getHeading());

        }
        path.setBrakingStrength(0.8);
        paths.add(path);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.update();
        intake.update();
        shooter.update();
        autonomousPathUpdate();

        telemetry.addData("path state", state);
        telemetry.addData("routine", routine);
        telemetry.addData("current step", currentRoutineIndex);
        telemetry.addData("current action", currentAction);
        telemetry.addData("current running path", nextPath);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("robot at end", robotAtEnd);
        telemetry.addData("is busy", isBusy);
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        robotAtEnd = follower.getCurrentTValue() > 0.97;

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

                follower.setMaxPower(SLOW_POWER);
                follower.followPath(getPath());

                setState(state + 1);
                break;

            case 1:
                if (follower.getCurrentTValue() > 0.4 && !ons) {
                    intake.setState(Intake.State.INTAKE_LAUNCH);
                    shooter.setState(Shooter.State.LAUNCH);
                    ons = true;
                }

                if (robotAtEnd && ons) {
                    follower.setMaxPower(MAX_POWER);
                    setState(state + 1);
                }
                break;

            case 2:
                if (!shooter.isBusy()) {
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
                shooter.setState(Shooter.State.READY);

                Path path = getPath();
                follower.followPath(path);

                setState(1);
                break;

            case 1:
                if(timer.getElapsedTimeSeconds() > 0.5) {
                    intake.setState(Intake.State.INTAKE_UP);
                    setState(2);
                }
                break;

            case 2:
                if (robotAtEnd && shooter.isGoalTargeted() || ons) {
                    if (!ons) {
                        timer.resetTimer();
                        ons = true;
                    }

                    if (timer.getElapsedTimeSeconds() > 0) {
                        intake.setState(Intake.State.INTAKE_LAUNCH);
                        shooter.setState(Shooter.State.LAUNCH);

                        setState(3);
                    }
                }
                break;

            case 3:
                if (!shooter.isBusy()) {
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
                intake.setState(Intake.State.INTAKE);
                follower.followPath(getPath());
                setState(1);
                break;

            case 1:
                if (robotAtEnd || shooter.areBallsCollected()) {
                    setState(2);
                }
                break;

            case 2:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 0) {
                    return false;
                }
                break;
        }

        return true;
    }

    private boolean gate() {
        switch (state) {
            case 0:
                intake.setState(Intake.State.INTAKE);
                follower.setMaxPower(SLOW_POWER);
                follower.followPath(getPath());

                if(currentAction == 31) {
                    setState(1);
                } else {
                    setState(2);
                }
                break;

            case 1:
                if (robotAtEnd || ons) {
                    if(!ons) {
                        timer.resetTimer();
                        ons = true;
                    }

                    if (timer.getElapsedTimeSeconds() > 0.5) {
                        follower.followPath(getPath());
                        setState(3);
                    }
                }
                break;

            case 2:
                if (robotAtEnd) {
                    follower.setMaxPower(MAX_POWER);
                    setState(3);
                }
                break;

            case 3:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 2) {
                    return false;
                }
                break;
        }

        return true;
    }

    private boolean hp() {
        switch (state) {
            case 0:
                intake.setState(Intake.State.INTAKE);
                follower.followPath(getPath());
                setState(1);
                break;

            case 1:
                if (robotAtEnd || shooter.areBallsCollected()) {
                    setState(2);
                }
                break;

            case 2:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 1.5) {
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
            intake.setState(Intake.State.INTAKE_UP);
            shooter.setState(Shooter.State.OFF);
            follower.followPath(getPath());

            ons = true;
        }
    }

    private Path getPath() {
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
        TransferConstants.endTurretPos = (int) (hardware.turret.getCurrentPosition() - shooter.getTurretReset());
    }
}