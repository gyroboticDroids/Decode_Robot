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

    private final PIDFController turretPIDFController;
    private final Hardware hardware;

    private State state;
    private final Timer timer;

    private final Vector goalToRobotVector = new Vector();

    public double goalXOffset = 0;
    public double goalYOffset = 0;

    public double targetPos;
    private double error = 0;

    private boolean isBusy = false;

    public boolean runTurret = true;

    private boolean timerReset = true;
    private double turretReset;

    private Vector launchVector = new Vector();

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
                    hardware.door.setPosition(ShooterConstants.DOOR_OPEN);
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
                    hardware.turret.setPower(-0.45);
                }
                break;
        }

        if (state != State.OFF && state != State.RESET) {
            targetGoal();
        } else {
            hardware.flywheel.setVelocity(ShooterConstants.FLYWHEEL_OFF);
            hardware.flywheel2.setVelocity(ShooterConstants.FLYWHEEL_OFF);
            hardware.hood.setPosition(ShooterConstants.HOOD_MIN_ANGLE);
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

        goalToRobotVector.setOrthogonalComponents(ShooterConstants.getGoalPos().getX() - robotPos.getX()
                + goalXOffset, ShooterConstants.getGoalPos().getY() - robotPos.getY() + goalYOffset);

        launchVector = calculateShotVectorAndUpdateTurret(robotPos.getHeading());

        hardware.flywheel.setVelocity(rampUpFlywheel(ShooterConstants.getFlywheelTicksFromVelocity(launchVector.getMagnitude(), launchVector.getTheta())));
        hardware.flywheel2.setVelocity(rampUpFlywheel(ShooterConstants.getFlywheelTicksFromVelocity(launchVector.getMagnitude(), launchVector.getTheta())));

        hardware.hood.setPosition(ShooterConstants.getHoodTicksFromDegrees(Math.toDegrees(launchVector.getTheta())));
    }

    private Vector calculateShotVectorAndUpdateTurret(double robotHeading) {
        //constants
        double g = 32.174 * 12;
        double x = goalToRobotVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
        double y = ShooterConstants.SCORE_HEIGHT;
        double a = ShooterConstants.SCORE_ANGLE;

        //calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), ShooterConstants.HOOD_MAX_ANGLE,
                ShooterConstants.HOOD_MIN_ANGLE);

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        //get robot velocity and convert it into parallel and perpendicular components
        Vector robotVelocity = hardware.poseTracker.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - goalToRobotVector.getTheta();

        if(coordinateTheta > 2 * Math.PI)
            coordinateTheta -= 2 * Math.PI;
        else if (coordinateTheta < -2 * Math.PI)
            coordinateTheta += 2 * Math.PI;

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        //recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), ShooterConstants.HOOD_MAX_ANGLE,
                ShooterConstants.HOOD_MIN_ANGLE);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / nvr);
        double turretAngle = Math.toDegrees(robotHeading - goalToRobotVector.getTheta() + turretVelCompOffset);

        if (turretAngle > 180) {
            turretAngle -= 360;
        }

        if (turretAngle > 185) {
            turretAngle -= 360;
        } else if (turretAngle < -185) {
            turretAngle += 360;
        }

        turretMoveTo(turretAngle);

        return new Vector(flywheelSpeed, hoodAngle);
    }

    private void turretMoveTo(double angle) {
        targetPos = MathFunctions.clamp(angle, ShooterConstants.TURRET_MIN_ANGLE, ShooterConstants.TURRET_MAX_ANGLE)
                * ShooterConstants.TURRET_TICKS_PER_DEGREE + turretReset;

        error = targetPos - hardware.turret.getCurrentPosition();

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
                ShooterConstants.getFlywheelTicksFromVelocity(launchVector.getMagnitude(), launchVector.getTheta()),
                ShooterConstants.FLYWHEEL_ACCURACY);
    }

    public boolean flywheelUpToSpeed(double accuracy) {
        return MathFunctions.roughlyEquals(hardware.flywheel.getVelocity(),
                ShooterConstants.getFlywheelTicksFromVelocity(launchVector.getMagnitude(), launchVector.getTheta()),
                accuracy);
    }

    public double getHoodAngle() {
        return launchVector.getTheta();
    }

    public double getFlywheelSpeed() {
        return launchVector.getMagnitude();
    }

    public boolean isBusy() {
        return isBusy;
    }

    private double rampUpFlywheel(double speed) {
        return MathFunctions.clamp(speed, hardware.flywheel.getVelocity() - ShooterConstants.FLYWHEEL_RAMP_SPEED,
                hardware.flywheel.getVelocity() + ShooterConstants.FLYWHEEL_RAMP_SPEED);
    }

    public Vector getGoalVector() {
        return goalToRobotVector;
    }

    public double getTurretReset() {
        return turretReset;
    }

    public boolean isGoalTargeted() {
        return Math.abs(error) < 3 && flywheelUpToSpeed();
    }
}
