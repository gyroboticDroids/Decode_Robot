package org.firstinspires.ftc.teamcode.robot.subassamblies;

import org.firstinspires.ftc.teamcode.robot.constants.IntakeConstants;

public class Intake {
    public enum State{
        INTAKE, CLEAR, INTAKE_UP, INTAKE_SLEEP
    }

    Hardware hardware;

    private State state;
    private boolean isBusy = false;

    public Intake(Hardware hardware){
        this.hardware = hardware;
    }

    public void update(){
        switch (state){
            case INTAKE:
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_LEFT_DOWN);
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_RIGHT_DOWN);

                hardware.intake.setPower(IntakeConstants.INTAKE_FORWARD);
                isBusy = false;
                break;

            case CLEAR:
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_LEFT_DOWN);
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_RIGHT_DOWN);

                hardware.intake.setPower(IntakeConstants.INTAKE_BACKWARD);
                isBusy = false;
                break;

            case INTAKE_UP:
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_LEFT_UP);
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_RIGHT_UP);

                hardware.intake.setPower(IntakeConstants.INTAKE_SLOW);
                isBusy = false;
                break;

            case INTAKE_SLEEP:
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_LEFT_UP);
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_RIGHT_UP);

                hardware.intake.setPower(0);
                isBusy = false;
                break;
        }
    }

    public void setState(State s){
        state = s;
        isBusy = true;
    }

    public boolean isBusy() {
        return isBusy;
    }

    public State getState() {
        return state;
    }
}