package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.control.PIDFController;
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
    public double parallelComponent = 0;

    private final PIDFController turretPIDFController;
    private final Hardware hardware;

    private State state;
    private final Timer timer;

    private Vector goalToRobotVector = new Vector();

    private boolean isBusy = false;

    public boolean velComp = false;
    public boolean runTurret = true;

    private boolean timerReset = true;

    private double turretOffset;
    private double turretReset;

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
        turretPIDFController = new PIDFController(ShooterConstants.TURRET_PIDF);
    }

    public void update() {
        switch (state) {
            case READY:
                hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);

                isBusy = false;
                break;

            case LAUNCH:
                if (!timerReset && (isLastBall() && timer.getElapsedTimeSeconds() > 0.5 || timer.getElapsedTimeSeconds() > 2)) {
                    timer.resetTimer();
                    timerReset = true;
                }

                if (timerReset && timer.getElapsedTimeSeconds() > 0.3) {
                    hardware.door.setPosition(ShooterConstants.DOOR_OPEN);
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_UP);
                }

                if (timerReset && timer.getElapsedTimeSeconds() > 0.4) {
                    hardware.door.setPosition(ShooterConstants.DOOR_CLOSED);
                    hardware.launcher.setPosition(ShooterConstants.LAUNCHER_DOWN);
                    isBusy = false;
                } else {
//                    if(!timerReset) {
//                        hardware.door.setPosition(flywheelUpToSpeed() ? ShooterConstants.DOOR_OPEN : ShooterConstants.DOOR_CLOSED);
//                    } else {
                        hardware.door.setPosition(ShooterConstants.DOOR_OPEN);
//                    }
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

                if (timer.getElapsedTimeSeconds() > 1) {
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
            hardware.flywheel2.setVelocity(ShooterConstants.FLYWHEEL_OFF);
            hardware.hood.setPosition(ShooterConstants.hoodAngle(0));
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

        goalToRobotVector.setOrthogonalComponents(ShooterConstants.getGoalPos().getX() - robotPos.getX(),
                ShooterConstants.getGoalPos().getY() - robotPos.getY());

        if (velComp) {
            Vector robotVelocity = hardware.poseTracker.getVelocity();

            double coordinateTheta = robotVelocity.getTheta() - goalToRobotVector.getTheta();

            if(coordinateTheta > 2 * Math.PI)
                coordinateTheta -= 2 * Math.PI;
            else if (coordinateTheta < -2 * Math.PI)
                coordinateTheta += 2 * Math.PI;

            parallelComponent = Math.cos(coordinateTheta) * robotVelocity.getMagnitude();

            robotVelocity.setMagnitude(parallelComponent * ShooterConstants.VELOCITY_TIME_MULTIPLIER +
                    robotVelocity.getMagnitude() * ShooterConstants.launchTime(goalToRobotVector.getMagnitude()));

            goalToRobotVector = goalToRobotVector.minus(robotVelocity);
        }

        double flywheelSpeed = ShooterConstants.flywheelSpeed(goalToRobotVector.getMagnitude());
        double hoodAngle = ShooterConstants.hoodAngle(goalToRobotVector.getMagnitude());
        double turretAngle = -Math.toDegrees(goalToRobotVector.getTheta()) + Math.toDegrees(robotPos.getHeading()) + turretOffset;

        if (turretAngle > 180) {
            turretAngle -= 360;
        }

        if (turretAngle > 185) {
            turretAngle -= 360;
        } else if (turretAngle < -185) {
            turretAngle += 360;
        }

        turretMoveTo(turretAngle);

        hardware.flywheel.setVelocity(rampUpFlywheel(flywheelSpeed));
        hardware.flywheel2.setVelocity(rampUpFlywheel(flywheelSpeed));
        hardware.hood.setPosition(hoodAngle);
    }

    private void turretMoveTo(double angle) {
        double targetPos = MathFunctions.clamp(angle, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE)
                * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretReset;

        double error = targetPos - hardware.turret.getCurrentPosition();

        turretPIDFController.updateError(error);

        double motorPower = MathFunctions.clamp(turretPIDFController.run(),
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
        return MathFunctions.roughlyEquals(hardware.flywheel.getVelocity(),
                ShooterConstants.flywheelSpeed(goalToRobotVector.getMagnitude()),
                ShooterConstants.FLYWHEEL_ACCURACY);
    }

    public boolean isBusy() {
        return isBusy;
    }

    public void turretOffset(double offset) {
        turretOffset += offset;
        turretOffset = MathFunctions.clamp(turretOffset, -180, 180);
    }

    private double rampUpFlywheel(double speed) {
        return MathFunctions.clamp(speed, hardware.flywheel.getVelocity() - ShooterConstants.FLYWHEEL_RAMP_SPEED,
                hardware.flywheel.getVelocity() + ShooterConstants.FLYWHEEL_RAMP_SPEED);
    }

    public Vector getGoalVector() {
        return goalToRobotVector;
    }

    public double getTurretOffset() {
        return turretOffset;
    }

    public double getTurretReset() {
        return turretReset;
    }
}
