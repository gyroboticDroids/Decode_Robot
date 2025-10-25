package org.firstinspires.ftc.teamcode.robot.subassamblies;


import com.pedropathing.util.Timer;

public class Shooter {
    public enum State {
        SLEEP, READY, LAUNCH, REJECT, PARK
    }

    private Hardware hardware;

    private State state;
    private Timer timer;

    private boolean ons = false;

    public Shooter(Hardware hardware) {
        this.hardware = hardware;

        timer = new Timer();
    }

    public void update() {
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
}
