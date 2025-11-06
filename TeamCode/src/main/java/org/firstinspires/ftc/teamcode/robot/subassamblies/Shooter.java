package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;

public class Shooter {
    public enum State {
        READY, LAUNCH, REJECT, OFF, RESET
    }

    private final Hardware hardware;

    private State state;
    private final Timer timer;

    private double goalDist = 0;

    private boolean isBusy = false;

    public boolean velComp = true;
    public boolean timerReset = true;

    private double turretOffset;
    private int turretReset = 0;

    //Ball detection
    boolean ball1 = false;
    boolean ball2 = false;
    boolean ball3 = false;

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
    }

    public void update() {
        switch (state) {
            case READY:
                hardware.flywheel.setVelocity(ShooterConstants.flywheelSpeed(goalDist));

                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                if (MathFunctions.roughlyEquals(hardware.flywheel.getVelocity(), ShooterConstants.flywheelSpeed(goalDist),
                        ShooterConstants.FLYWHEEL_ACCURACY) || timer.getElapsedTimeSeconds() > 1)
                    isBusy = false;
                break;

            case LAUNCH:
                hardware.flywheel.setVelocity(ShooterConstants.flywheelSpeed(goalDist));

                boolean ballDetection = areBallsClear();

                if (!ballDetection) {
                    hardware.door.setPosition(ShooterConstants.DOOR_OPEN);
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                }

                if (ballDetection) {
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                    timer.resetTimer();
                    timerReset = true;
                }

                if (timerReset && timer.getElapsedTimeSeconds() > 0.5) {
                    hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                    isBusy = false;
                }
                break;

            case REJECT:
                hardware.flywheel.setVelocity(ShooterConstants.FLYWHEEL_REJECT);

                if (timer.getElapsedTimeSeconds() > 1.5) {
                    hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);
                    isBusy = false;
                } else
                    hardware.door.setPosition(ShooterConstants.DOOR_OPEN);

                if (timer.getElapsedTimeSeconds() > 1.5)
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                else if (timer.getElapsedTimeSeconds() > 0.5)
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_UP);
                break;

            case OFF:
                hardware.flywheel.setVelocity(ShooterConstants.FLYWHEEL_OFF);

                turretMoveTo(0);

                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                isBusy = false;
                break;

            case RESET:
                hardware.flywheel.setVelocity(ShooterConstants.FLYWHEEL_OFF);

                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                hardware.turret.setPower(0.5);

                if (timer.getElapsedTimeSeconds() > 1 || hardware.turret.getVelocity() < 10) {
                    hardware.turret.setPower(0);
                    turretReset = ShooterConstants.TURRET_RESET_POS - hardware.turret.getCurrentPosition();
                    isBusy = false;
                }
                break;
        }

        if (state != State.OFF && state != State.RESET) {
            targetGoal();
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

        goalDist = Math.sqrt(Math.pow(robotPos.getX() - ShooterConstants.GOAL_POS.getX(), 2)
                + Math.pow(robotPos.getY() - ShooterConstants.GOAL_POS.getY(), 2));

        if (velComp) {
            Vector velocity = hardware.poseTracker.getVelocity();
            Vector ballDist = new Vector(velocity.getMagnitude() * ShooterConstants.launchTime(goalDist),
                    velocity.getTheta());

            robotPos.getAsVector().plus(ballDist);
        }

        double angle = Math.atan((robotPos.getX() - ShooterConstants.GOAL_POS.getX())
                / (robotPos.getY() - ShooterConstants.GOAL_POS.getY())) - robotPos.getHeading() + turretOffset;

        while (Math.abs(angle) > 180) {
            if (angle > 180) {
                angle -= 360;
            } else if (angle < -180) {
                angle += 360;
            }
        }

        turretMoveTo(angle);

        hardware.hood.setPosition(MathFunctions.clamp(ShooterConstants.hoodAngle(goalDist), 0, 1));
    }

    private void turretMoveTo(double angle) {
        double targetPos = angle * ShooterConstants.TURRET_TICKS_PER_DEGREE - turretReset;

        double error = targetPos - hardware.turret.getCurrentPosition();

        double motorPower = MathFunctions.clamp(error * ShooterConstants.TURRET_P_GAIN, -1, 1);

        hardware.turret.setPower(motorPower);
    }

    private void ballDetectionUpdate() {
        if (hardware.ball1.getDistance(DistanceUnit.INCH) < 1)
            ball1Timer.resetTimer();
        ball1 = ball1Timer.getElapsedTimeSeconds() < ShooterConstants.BALL_DETECTION_TIME;

        if (hardware.ball2.getDistance(DistanceUnit.INCH) < 1)
            ball2Timer.resetTimer();
        ball2 = ball2Timer.getElapsedTimeSeconds() < ShooterConstants.BALL_DETECTION_TIME;

        if (hardware.ball3.getDistance(DistanceUnit.INCH) < 1)
            ball3Timer.resetTimer();
        ball3 = ball3Timer.getElapsedTimeSeconds() < ShooterConstants.BALL_DETECTION_TIME;
    }

    public boolean areBallsClear() {
        return !ball1 && !ball2 && !ball3;
    }

    public boolean areBallsCollected() {
        return ball1 && ball2 && ball3;
    }

    public boolean isBusy() {
        return isBusy;
    }

    public void turretOffset(double offset) {
        turretOffset += offset;
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
