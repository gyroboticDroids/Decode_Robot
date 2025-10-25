package org.firstinspires.ftc.teamcode.robot.subassamblies;


import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;

public class Shooter {
    public enum State {
        SLEEP, READY, LAUNCH, REJECT, PARK
    }

    private Hardware hardware;

    private State state;
    private Timer timer;

    private double goalDist = 0;

    private boolean ons = false;

    public Shooter(Hardware hardware) {
        this.hardware = hardware;

        timer = new Timer();
    }

    public void update() {
        targetGoal();

        switch (state) {
            case SLEEP:
                break;

            case READY:
                break;

            case LAUNCH:
                break;

            case REJECT:
                break;

            case PARK:
                break;
        }

        ons = false;
    }

    public void setState(State s) {
        state = s;

        ons = true;
        timer.resetTimer();
    }

    private void targetGoal() {
        Pose robotPos = hardware.poseTracker.getPose();

        goalDist = Math.sqrt(Math.pow(robotPos.getX() - ShooterConstants.GOAL_POS.getX(), 2)
                + Math.pow(robotPos.getY() - ShooterConstants.GOAL_POS.getY(), 2));

        double angle = Math.atan((robotPos.getX() - ShooterConstants.GOAL_POS.getX())
                / (robotPos.getY() - ShooterConstants.GOAL_POS.getY())) - robotPos.getHeading();
    }
}
