package org.firstinspires.ftc.teamcode.robot.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Hardware;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Intake;
import org.firstinspires.ftc.teamcode.robot.subassamblies.Shooter;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "run auto", group = "close auto", preselectTeleOp = "Master Tele-op")
public class RunAuto extends OpMode {
    private List<Integer> routine;
    private int currentRoutineIndex = 0;
    int currentAction = 0;
    private int nextPath = -1;

    private Pose startClose = new Pose(115.5, 123, Math.toRadians(45)),
            startFar = new Pose(85.5, 6.5, Math.toRadians(0)),
            scoreClose = new Pose(100, 100, Math.toRadians(0)),
            scoreMiddle = new Pose(86, 78, Math.toRadians(0)),
            scoreFar = new Pose(84, 19, Math.toRadians(0)),
            balls1 = new Pose(118, 82.5, Math.toRadians(0)),
            balls2 = new Pose(122, 59, Math.toRadians(0)),
            balls3 = new Pose(123, 35, Math.toRadians(0)),
            gateReady = new Pose(115, 63, Math.toRadians(0)),
            gateBump = new Pose(121, 65, Math.toRadians(0)),
            gateCollect = new Pose(130, 56, Math.toRadians(36)),
            hpReady1 = new Pose(128, 12, Math.toRadians(0)),
            hpReady2 = new Pose(108.25, 8, Math.toRadians(0)),
            hpPreset = new Pose(128, 8, Math.toRadians(0)),
            hpGate = new Pose(127.5, 19, Math.toRadians(45)),
            endClose = new Pose(120, 70, Math.toRadians(270)),
            endFar = new Pose(105, 33, Math.toRadians(0));

    private Pose controlBalls1 = new Pose(95, 82),
            controlBalls2 = new Pose(90, 55),
            controlBalls3 = new Pose(92, 31),
            controlBalls1Far = new Pose(95, 89),
            controlBalls2Far = new Pose(90, 62),
            controlBalls3Far = new Pose(92, 35),
            controlGate = new Pose(100, 63),
            controlHp1 = new Pose(105, 10),
            controlHp3 = new Pose(127.5, 9);

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

    List<LynxModule> allHubs;

    @Override
    public void init() {
        timer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.95);

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
        startClose = ShooterConstants.mirror(startClose);
        startFar = ShooterConstants.mirror(startFar);
        scoreClose = ShooterConstants.mirror(scoreClose);
        scoreMiddle = ShooterConstants.mirror(scoreMiddle);
        scoreFar = ShooterConstants.mirror(scoreFar);
        balls1 = ShooterConstants.mirror(balls1);
        balls2 = ShooterConstants.mirror(balls2);
        balls3 = ShooterConstants.mirror(balls3);
        gateReady = ShooterConstants.mirror(gateReady);
        gateBump = ShooterConstants.mirror(gateBump);
        gateCollect = ShooterConstants.mirror(gateCollect);
        hpReady1 = ShooterConstants.mirror(hpReady1);
        hpReady2 = ShooterConstants.mirror(hpReady2);
        hpPreset = ShooterConstants.mirror(hpPreset);
        hpGate = ShooterConstants.mirror(hpGate);
        endClose = ShooterConstants.mirror(endClose);
        endFar = ShooterConstants.mirror(endFar);

        controlBalls1 = ShooterConstants.mirror(controlBalls1);
        controlBalls2 = ShooterConstants.mirror(controlBalls2);
        controlBalls3 = ShooterConstants.mirror(controlBalls3);
        controlBalls1Far = ShooterConstants.mirror(controlBalls1Far);
        controlBalls2Far = ShooterConstants.mirror(controlBalls2Far);
        controlBalls3Far = ShooterConstants.mirror(controlBalls3Far);
        controlGate = ShooterConstants.mirror(controlGate);
        controlHp1 = ShooterConstants.mirror(controlHp1);
        controlHp3 = ShooterConstants.mirror(controlHp3);
    }

    private void buildPaths() {
        Pose lastControlPoint = null;
        Pose lastPose = startPose;

        PathChain path;

        boolean scoringFar = false;

        for (int i = 1; i < routine.size(); i++) {
            switch (routine.get(i)) {
                case 10:
                    path = follower.pathBuilder()
                            .addPath((lastControlPoint == null) ? new BezierLine(lastPose, scoreClose) :
                                    new BezierCurve(lastPose, lastControlPoint, scoreClose))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), scoreClose.getHeading(), 0.8)
                            .build();

                    paths.add(path);

                    scoringFar = false;
                    lastControlPoint = null;
                    break;

                case 11:
                    path = follower.pathBuilder()
                            .addPath((lastControlPoint == null) ? new BezierLine(lastPose, scoreMiddle) :
                                    new BezierCurve(lastPose, lastControlPoint, scoreMiddle))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), scoreMiddle.getHeading(), 0.4)
                            .build();

                    paths.add(path);

                    scoringFar = false;
                    lastControlPoint = null;
                    break;

                case 12:
                    path = follower.pathBuilder()
                            .addPath((lastControlPoint == null) ? new BezierLine(lastPose, scoreFar) :
                                    new BezierCurve(lastPose, lastControlPoint, scoreFar))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), scoreFar.getHeading())
                            .setBrakingStart(4)
                            .setBrakingStrength(0.7)
                            .build();


                    paths.add(path);

                    scoringFar = true;
                    lastControlPoint = null;
                    break;

                case 20:
                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, scoringFar ? controlBalls1Far : controlBalls1, balls1))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), balls1.getHeading(), 0.5)
                            .build();

                    paths.add(path);

                    lastControlPoint = scoringFar ? controlBalls1Far : null;
                    break;

                case 21:
                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, scoringFar ? controlBalls2Far : controlBalls2, balls2))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), balls2.getHeading(), 0.7)
                            .build();

                    paths.add(path);

                    lastControlPoint = scoringFar ? controlBalls2Far : null;
                    break;

                case 22:
                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, scoringFar ? controlBalls2Far : controlBalls2, gateBump))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), gateBump.getHeading(), 0.7)
                            .build();

                    paths.add(path);

                    lastControlPoint = scoringFar ? controlBalls2Far : null;
                    break;

                case 23:
                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, scoringFar ? controlBalls3Far : controlBalls3, balls3))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), balls3.getHeading(), 0.7)
                            .build();

                    paths.add(path);

                    lastControlPoint = scoringFar ? controlBalls3Far : null;
                    break;

                case 30:
                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlGate, gateReady))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), gateReady.getHeading(), 0.7)
                            .setBrakingStrength(1.2)
                            .build();

                    paths.add(path);

                    path = follower.pathBuilder()
                            .addPath(new BezierLine(gateReady, gateCollect))
                            .setLinearHeadingInterpolation(gateReady.getHeading(), gateCollect.getHeading(), 0.7)
                            .build();

                    paths.add(path);

                    lastControlPoint = controlGate;
                    break;

                case 31:
                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlGate, gateReady))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), gateReady.getHeading(), 0.7)
                            .setBrakingStrength(1.2)
                            .build();

                    paths.add(path);

                    path = follower.pathBuilder()
                            .addPath(new BezierLine(gateReady, gateBump))
                            .setLinearHeadingInterpolation(gateReady.getHeading(), gateBump.getHeading(), 0.7)
                            .build();

                    paths.add(path);

                    path = follower.pathBuilder()
                            .addPath(new BezierLine(gateBump, gateCollect))
                            .setLinearHeadingInterpolation(gateBump.getHeading(), gateCollect.getHeading())
                            .build();

                    paths.add(path);

                    lastControlPoint = controlGate;
                    break;

                case 40:
                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlHp1, hpReady1))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), hpReady1.getHeading())
                            .setBrakingStart(4)
                            .setBrakingStrength(0.5)
                            .build();

                    paths.add(path);

                    path = follower.pathBuilder()
                            .addPath(new BezierLine(hpReady1, hpPreset))
                            .setLinearHeadingInterpolation(hpReady1.getHeading(), hpPreset.getHeading())
                            .build();

                    paths.add(path);

                    lastControlPoint = controlHp1;
                    break;

                case 41:
                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(lastPose, controlHp1, hpReady2))
                            .setLinearHeadingInterpolation(lastPose.getHeading(), hpReady2.getHeading())
                            .build();

                    paths.add(path);

                    path = follower.pathBuilder()
                            .addPath(new BezierCurve(hpReady2, controlHp3, hpGate))
                            .setLinearHeadingInterpolation(hpReady2.getHeading(), hpGate.getHeading())
                            .build();

                    paths.add(path);

                    lastControlPoint = controlHp1;
                    break;
            }

            lastPose = paths.get(paths.size() - 1).lastPath().endPose();
        }

        if (lastPose.distanceFrom(scoreFar) < 1) {
            path = follower.pathBuilder()
                    .addPath(new BezierLine(lastPose, endFar))
                    .setLinearHeadingInterpolation(lastPose.getHeading(), endFar.getHeading())
                    .build();
        } else {
            path = follower.pathBuilder()
                    .addPath(new BezierLine(lastPose, endClose))
                    .setLinearHeadingInterpolation(lastPose.getHeading(), endClose.getHeading())
                    .build();
        }

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
        telemetry.addData("is goal targeted", shooter.isGoalTargeted());
        telemetry.update();
    }

    private void autonomousPathUpdate() {
        robotAtEnd = follower.getCurrentTValue() > 0.97;

        if (!isBusy) {
            nextAction();
        }

        if (currentAction >= 10 && currentAction <= 19) {
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

                follower.followPath(getPath(), false);

                setState(state + 1);
                break;

            case 1:
                if (follower.getCurrentTValue() > 0.4 && !ons && shooter.flywheelUpToSpeed(150)) {
                    intake.setState(Intake.State.INTAKE_LAUNCH);
                    shooter.setState(Shooter.State.LAUNCH);
                    ons = true;
                }

                if (ons) {
                    setState(state + 1);
                }
                break;

            case 2:
                if (!shooter.isBusy() && follower.getCurrentTValue() > 0.9) {
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

                PathChain path = getPath();
                follower.followPath(path, true);

                setState(1);
                break;

            case 1:
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    intake.setState(Intake.State.INTAKE_UP);
                    setState(2);
                }
                break;

            case 2:
                if (!follower.isBusy() && shooter.isGoalTargeted()) {
                    intake.setState(Intake.State.INTAKE_LAUNCH);
                    shooter.setState(Shooter.State.LAUNCH);

                    setState(3);
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
                follower.followPath(getPath());

                setState(1);
                break;

            case 1:
                if (robotAtEnd) {
                    follower.followPath(getPath(), 0.7,true);

                    if (currentAction == 31) {
                        setState(2);
                    } else {
                        setState(3);
                    }
                }
                break;

            case 2:
                if (robotAtEnd || ons) {
                    if (!ons) {
                        timer.resetTimer();
                        ons = true;
                    }

                    if (timer.getElapsedTimeSeconds() > 0.5) {
                        follower.followPath(getPath());
                        setState(3);
                    }
                }
                break;

            case 3:
                if (robotAtEnd || timer.getElapsedTimeSeconds() > 1.5) {
                    setState(4);
                }
                break;

            case 4:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 1.5) {
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
                if (follower.getCurrentTValue() > 0.9) {
                    follower.followPath(getPath());
                    setState(2);
                }
                break;

            case 2:
                if (robotAtEnd || shooter.areBallsCollected()) {
                    setState(3);
                }
                break;

            case 3:
                if (shooter.areBallsCollected() || timer.getElapsedTimeSeconds() > 1) {
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
        TransferConstants.endTurretPos = (int) (hardware.turret.getCurrentPosition() - shooter.getTurretReset());
    }
}