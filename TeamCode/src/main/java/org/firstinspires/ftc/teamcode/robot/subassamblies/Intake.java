package org.firstinspires.ftc.teamcode.robot.subassamblies;

import org.firstinspires.ftc.teamcode.robot.constants.IntakeConstants;

public class Intake {
    public enum State{
        INTAKE, CLEAR, INTAKE_UP
    }

    Hardware hardware;

    private State state;

    public Intake(Hardware hardware){
        this.hardware = hardware;
    }

    public void update(){
        switch (state){
            case INTAKE:
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_LEFT_DOWN);
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_RIGHT_DOWN);

                hardware.intake.setPower(IntakeConstants.INTAKE_FORWARD);
                break;

            case CLEAR:
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_LEFT_DOWN);
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_RIGHT_DOWN);

                hardware.intake.setPower(IntakeConstants.INTAKE_BACKWARD);
                break;

            case INTAKE_UP:
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_LEFT_UP);
                hardware.intakePivotLeft.setPosition(IntakeConstants.INTAKE_PIVOT_RIGHT_UP);

                hardware.intake.setPower(IntakeConstants.INTAKE_FORWARD);
                break;
        }
    }

    public void setState(State s){
        state = s;
    }
}