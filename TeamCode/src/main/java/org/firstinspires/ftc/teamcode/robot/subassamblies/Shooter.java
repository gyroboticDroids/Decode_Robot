package org.firstinspires.ftc.teamcode.robot.subassamblies;


import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;

public class Shooter {
    public enum State {
        SLEEP, READY, LAUNCH, REJECT, PARK
    }

    private final Hardware hardware;

    private State state;
    private final Timer timer;

    private double goalDist = 0;

    private boolean isBusy = false;

    public Shooter(Hardware hardware) {
        this.hardware = hardware;

        timer = new Timer();
    }

    public void update() {
        targetGoal();

        switch (state) {
            case SLEEP:
                hardware.flywheel.setVelocity(ShooterConstants.FLYWHEEL_OFF);

                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                isBusy = false;
                break;

            case READY:
                hardware.flywheel.setVelocity(ShooterConstants.flywheelSpeed(goalDist));

                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                if (MathFunctions.roughlyEquals(hardware.flywheel.getVelocity(), ShooterConstants.flywheelSpeed(goalDist),
                        ShooterConstants.FLYWHEEL_ACCURACY))
                    isBusy = false;
                break;

            case LAUNCH:
                hardware.flywheel.setVelocity(ShooterConstants.flywheelSpeed(goalDist));

                if (timer.getElapsedTimeSeconds() > 1) {
                    hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);
                    isBusy = false;
                }
                else {
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
                }
                else
                    hardware.door.setPosition(ShooterConstants.DOOR_OPEN);

                if (timer.getElapsedTimeSeconds() > 1.5)
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                else if(timer.getElapsedTimeSeconds() > 0.5)
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_UP);
                break;

            case PARK:
                hardware.flywheel.setVelocity(ShooterConstants.FLYWHEEL_OFF);

                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                isBusy = false;
                break;
        }
    }

    public void setState(State s) {
        state = s;
        isBusy = true;
        timer.resetTimer();
    }

    private void targetGoal() {
        Pose robotPos = hardware.poseTracker.getPose();

        goalDist = Math.sqrt(Math.pow(robotPos.getX() - ShooterConstants.GOAL_POS.getX(), 2)
                + Math.pow(robotPos.getY() - ShooterConstants.GOAL_POS.getY(), 2));

        double angle = Math.atan((robotPos.getX() - ShooterConstants.GOAL_POS.getX())
                / (robotPos.getY() - ShooterConstants.GOAL_POS.getY())) - robotPos.getHeading();

        //p gain for turret

        hardware.hood.setPosition(ShooterConstants.hoodAngle(goalDist));
    }
}
