package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;

public class Shooter {
    public enum State {
        READY, LAUNCH, OFF, RESET
    }

    private final Hardware hardware;

    private State state;
    private final Timer timer;

    private double goalDist = 0;

    private boolean isBusy = false;

    public boolean velComp = false;
    public boolean runTurret = true;

    private boolean timerReset = true;

    private double turretOffset;
    private double turretReset = 0;

    //Ball detection
    private boolean ball1 = false;
    private boolean ball2 = false;
    private boolean ball3 = false;

    Timer ball1Timer;
    Timer ball2Timer;
    Timer ball3Timer;

    public Shooter(Hardware hardware) {
        this.hardware = hardware;

        timer = new Timer();

        //Ball detection
        ball1Timer = new Timer();
        ball2Timer = new Timer();
        ball3Timer = new Timer();

        ball1Timer.resetTimer();
        ball2Timer.resetTimer();
        ball3Timer.resetTimer();

        turretReset = -TransferConstants.endTurretPos;
    }

    public void update() {
        switch (state) {
            case READY:
                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                isBusy = false;
                break;

            case LAUNCH:
                boolean lastBall = isLastBall();

                if (!lastBall) {
                    hardware.door.setPosition(ShooterConstants.DOOR_OPEN);
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                }

                if (!timerReset && (lastBall && timer.getElapsedTimeSeconds() > 0.5 || timer.getElapsedTimeSeconds() > 2)) {
                    timer.resetTimer();
                    timerReset = true;
                }

                if (timerReset && timer.getElapsedTimeSeconds() > 0.25) {
                    hardware.door.setPosition(ShooterConstants.DOOR_OPEN);
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_UP);
                }

                if (timerReset && timer.getElapsedTimeSeconds() > 0.5) {
                    hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                    isBusy = false;
                }
                break;

            case OFF:
                turretMoveTo(0);

                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                isBusy = false;
                break;

            case RESET:
                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                if (timer.getElapsedTimeSeconds() > 1 && Math.abs(hardware.turret.getPower()) > 0.4) {
                    hardware.turret.setPower(0);
                    turretReset = hardware.turret.getCurrentPosition() - ShooterConstants.TURRET_RESET_POS;
                    isBusy = false;
                } else {
                    hardware.turret.setPower(-0.5);
                }
                break;
        }

        if (state != State.OFF && state != State.RESET) {
            targetGoal();
        } else {
            hardware.flywheel.setVelocity(ShooterConstants.FLYWHEEL_OFF);
        }

        ballDetectionUpdate();
    }

    public void setState(State s) {
        state = s;
        isBusy = true;
        timerReset = false;
        timer.resetTimer();
    }

    public State getState() {
        return state;
    }

    private void targetGoal() {
        Pose robotPos = hardware.poseTracker.getPose();

        goalDist = updateGoalDist(robotPos);

        if (velComp) {
            Vector velocity = hardware.poseTracker.getVelocity();
            Vector ballDist = new Vector(velocity.getMagnitude() * ShooterConstants.launchTime(goalDist),
                    velocity.getTheta());

            robotPos = new Pose(robotPos.getX() + ballDist.getXComponent(),
                    robotPos.getY() + ballDist.getYComponent(), robotPos.getHeading());

            goalDist = updateGoalDist(robotPos);
        }

        double robotHeading = Math.toDegrees(robotPos.getHeading());

        if (robotHeading > 180) {
            robotHeading -= 360;
        }

        double angle = -Math.toDegrees(Math.atan((robotPos.getY() - ShooterConstants.getGoalPos().getY())
                / (robotPos.getX() - ShooterConstants.getGoalPos().getX()))) + (TransferConstants.isAllianceColorRed ? 0 : 180)
                + robotHeading + turretOffset;

        if (angle > 185) {
            angle -= 360;
        } else if (angle < -185) {
            angle += 360;
        }

        turretMoveTo(angle);

        hardware.hood.setPosition(MathFunctions.clamp(ShooterConstants.hoodAngle(goalDist), 0, 1));
        hardware.flywheel.setVelocity(ShooterConstants.flywheelSpeed(goalDist));
    }

    private double updateGoalDist(Pose robotPos) {
        return Math.sqrt(Math.pow(robotPos.getX() - ShooterConstants.getGoalPos().getX(), 2)
                + Math.pow(robotPos.getY() - ShooterConstants.getGoalPos().getY(), 2));
    }

    private void turretMoveTo(double angle) {
        double targetPos = MathFunctions.clamp(angle, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE)
                * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretReset;

        double error = targetPos - hardware.turret.getCurrentPosition();

        double motorPower = MathFunctions.clamp(error * ShooterConstants.TURRET_P_GAIN,
                -ShooterConstants.TURRET_MAX_SPEED, ShooterConstants.TURRET_MAX_SPEED);

        hardware.turret.setPower(runTurret ? motorPower : 0);
    }

    private void ballDetectionUpdate() {
        if (hardware.ball1.getDistance(DistanceUnit.INCH) < ShooterConstants.BALL_DETECTION_DISTANCE)
            ball1Timer.resetTimer();
        ball1 = ball1Timer.getElapsedTimeSeconds() < ShooterConstants.BALL_DETECTION_TIME;

        if (hardware.ball2.getDistance(DistanceUnit.INCH) < ShooterConstants.BALL_DETECTION_DISTANCE)
            ball2Timer.resetTimer();
        ball2 = ball2Timer.getElapsedTimeSeconds() < ShooterConstants.BALL_DETECTION_TIME;

        if (hardware.ball3.getDistance(DistanceUnit.INCH) < ShooterConstants.BALL_DETECTION_DISTANCE)
            ball3Timer.resetTimer();
        ball3 = ball3Timer.getElapsedTimeSeconds() < ShooterConstants.BALL_DETECTION_TIME;
    }

    public boolean isLastBall() {
        return !ball1 && !ball2;
    }

    public boolean areBallsCollected() {
        return ball1 && ball2 && ball3;
    }

    public boolean flywheelUpToSpeed() {
        return MathFunctions.roughlyEquals(hardware.flywheel.getVelocity(), ShooterConstants.flywheelSpeed(goalDist),
                ShooterConstants.FLYWHEEL_ACCURACY);
    }

    public boolean isBusy() {
        return isBusy;
    }

    public void turretOffset(double offset) {
        turretOffset += offset;
        turretOffset = MathFunctions.clamp(turretOffset, -180, 180);
    }

    public double getGoalDist() {
        return goalDist;
    }

    public double getTurretOffset() {
        return turretOffset;
    }

    public double getTurretReset() {
        return turretReset;
    }
}
