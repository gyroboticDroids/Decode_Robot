package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.util.Timer;

public class Intake {
    public enum State{
        INTAKE, CLEAR, INTAKEUP
    }

    Hardware hardware;

    private State state;
    private Timer timer;

    public Intake(Hardware hardware){
        this.hardware = hardware;
    }

    public void update(){

    }

    public void setState(String s){

    }
}
