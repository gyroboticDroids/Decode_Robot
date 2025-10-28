package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;

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

    private double turretOffset;
    private int turretReset = 0;

    public Shooter(Hardware hardware) {
        this.hardware = hardware;

        timer = new Timer();
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

                if (timer.getElapsedTimeSeconds() > 1) {
                    hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);
                    isBusy = false;
                } else {
                    hardware.door.setPosition(ShooterConstants.DOOR_OPEN);
                }

                if (timer.getElapsedTimeSeconds() > 1.5)
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                else
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_UP);
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

                hardware.turret.setTargetPosition(-turretReset);

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
    }

    public void setState(State s) {
        state = s;
        isBusy = true;
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

        while (Math.abs(angle) >= 180) {
            if (angle > 180) {
                angle -= 360;
            } else if (angle < -180) {
                angle += 360;
            }
        }

        hardware.turret.setTargetPosition((int) (angle * ShooterConstants.TURRET_TICKS_PER_DEGREE - turretReset));

        hardware.hood.setPosition(MathFunctions.clamp(ShooterConstants.hoodAngle(goalDist), 0, 1));
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
}
