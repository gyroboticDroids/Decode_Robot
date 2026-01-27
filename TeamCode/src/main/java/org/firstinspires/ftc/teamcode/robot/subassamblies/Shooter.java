package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.robot.constants.TransferConstants;

public class Shooter extends ShooterConstants{
    public enum State {
        READY, LAUNCH, OFF, RESET
    }

    private final PIDFController turretPIDFController;
    private final PIDFController flywheelPIDFController;
    private final Hardware hardware;

    private State state;
    private final Timer timer;

    private final Vector robotToGoalVector = new Vector();

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
        turretPIDFController = new PIDFController(TURRET_PIDF);
        flywheelPIDFController = new PIDFController(FLYWHEEL_PIDF);
        flywheelPIDFController.updateFeedForwardInput(1);
    }

    public void update() {
        switch (state) {
            case READY:
                hardware.launcher.setPosition(LAUNCHER_DOWN);
                hardware.door.setPosition(DOOR_CLOSED);

                isBusy = false;
                break;

            case LAUNCH:
                if (!timerReset && (isNoBalls() && timer.getElapsedTimeSeconds() > 0.4 || timer.getElapsedTimeSeconds() > 1)) {
                    timer.resetTimer();
                    timerReset = true;
                }

                if (timerReset && timer.getElapsedTimeSeconds() > 0.1) {
                    hardware.door.setPosition(DOOR_CLOSED);
                    isBusy = false;
                } else {
                    hardware.door.setPosition(DOOR_OPEN);
                }
                break;

            case OFF:
                turretMoveTo(0);

                hardware.launcher.setPosition(LAUNCHER_DOWN);
                hardware.door.setPosition(DOOR_CLOSED);

                isBusy = false;
                break;

            case RESET:
                hardware.launcher.setPosition(LAUNCHER_DOWN);
                hardware.door.setPosition(DOOR_CLOSED);

                if (timer.getElapsedTimeSeconds() > 1) {
                    hardware.turret.setPower(0);
                    turretReset = hardware.turret.getCurrentPosition() - TURRET_RESET_POS;
                    isBusy = false;
                } else {
                    hardware.turret.setPower(-0.4);
                }
                break;
        }

        if (state != State.OFF && state != State.RESET) {
            targetGoal();
        } else {
            hardware.flywheel.setPower(FLYWHEEL_OFF);
            hardware.flywheel2.setPower(FLYWHEEL_OFF);
            hardware.hood.setPosition(getHoodTicksFromDegrees(Math.toDegrees(HOOD_MIN_ANGLE)));
            flywheelPIDFController.reset();
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
        Pose robotPos = hardware.follower.getPose();

        robotToGoalVector.setOrthogonalComponents(getGoalPos().getX() - robotPos.getX(),
                getGoalPos().getY() - robotPos.getY());

        launchVector = calculateShotVectorAndUpdateTurret(robotPos.getHeading());

        setFlywheelSpeed(rampUpFlywheel(getFlywheelTicksFromVelocity(launchVector.getMagnitude())));

        hardware.hood.setPosition(getHoodTicksFromDegrees(Math.toDegrees(launchVector.getTheta())));
    }

    private Vector calculateShotVectorAndUpdateTurret(double robotHeading) {
        //constants
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        //calculate initial launch components
        double hoodAngle = MathFunctions.clamp(Math.atan(2 * y / x - Math.tan(a)), HOOD_MAX_ANGLE,
                HOOD_MIN_ANGLE);

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        //get robot velocity and convert it into parallel and perpendicular components
        Vector robotVelocity = hardware.follower.getVelocity();

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        //velocity compensation variables
        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        //recalculate launch components
        hoodAngle = MathFunctions.clamp(Math.atan(vz / nvr), HOOD_MAX_ANGLE, HOOD_MIN_ANGLE);

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));

        //update turret
        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double turretAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + turretVelCompOffset);

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
        //for turret accuracy
        error = angle * TURRET_TICKS_PER_DEGREE + turretReset - hardware.turret.getCurrentPosition();

        targetPos = MathFunctions.clamp(angle, TURRET_MIN_ANGLE, TURRET_MAX_ANGLE) * TURRET_TICKS_PER_DEGREE + turretReset;

        double clampedError = targetPos - hardware.turret.getCurrentPosition();

        turretPIDFController.updateFeedForwardInput(Math.signum(Math.abs(clampedError) < TURRET_F_ERROR ? 0 : clampedError));
        turretPIDFController.updateError(clampedError);

        double motorPower = MathFunctions.clamp(turretPIDFController.run(), -TURRET_MAX_SPEED, TURRET_MAX_SPEED);

        hardware.turret.setPower(runTurret ? motorPower : 0);
    }

    private void setFlywheelSpeed(double speed) {
        double flywheelError = speed - hardware.flywheel.getVelocity();

        flywheelPIDFController.updateError(flywheelError);

        double flywheelPower = MathFunctions.clamp(flywheelPIDFController.run(), -1, 1);

        hardware.flywheel.setPower(flywheelPower);
        hardware.flywheel2.setPower(flywheelPower);
    }

    private void ballDetectionUpdate() {
        if (hardware.ball1.getDistance(DistanceUnit.INCH) < BALL_DETECTION_DISTANCE)
            ball1Timer.resetTimer();
        ball1 = ball1Timer.getElapsedTimeSeconds() < BALL_DETECTION_TIME;

        if (hardware.ball2.getDistance(DistanceUnit.INCH) < BALL_DETECTION_DISTANCE)
            ball2Timer.resetTimer();
        ball2 = ball2Timer.getElapsedTimeSeconds() < BALL_DETECTION_TIME;

        if (hardware.ball3.getDistance(DistanceUnit.INCH) < BALL_DETECTION_DISTANCE)
            ball3Timer.resetTimer();
        ball3 = ball3Timer.getElapsedTimeSeconds() < BALL_DETECTION_TIME;
    }

    public void resetPIDFS() {
        flywheelPIDFController.reset();
        turretPIDFController.reset();
    }

    private double rampUpFlywheel(double speed) {
        return MathFunctions.clamp(speed, hardware.flywheel.getVelocity() - FLYWHEEL_RAMP_SPEED,
                hardware.flywheel.getVelocity() + FLYWHEEL_RAMP_SPEED);
    }

    public boolean isNoBalls() {
        return !ball1 && !ball2 && !ball3;
    }

    public boolean areBallsCollected() {
        return ball1 && ball2 && ball3;
    }

    public boolean flywheelUpToSpeed() {
        return MathFunctions.roughlyEquals(hardware.flywheel.getVelocity(),
                getFlywheelTicksFromVelocity(launchVector.getMagnitude()), FLYWHEEL_ACCURACY);
    }

    public boolean flywheelUpToSpeed(double accuracy) {
        return MathFunctions.roughlyEquals(hardware.flywheel.getVelocity(),
                getFlywheelTicksFromVelocity(launchVector.getMagnitude()), accuracy);
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

    public Vector getGoalVector() {
        return robotToGoalVector;
    }

    public double getTurretReset() {
        return turretReset;
    }

    public boolean isGoalTargeted() {
        return Math.abs(error) < TURRET_TICKS_PER_DEGREE * Math.toDegrees(Math.atan(SCORE_ACCURACY /
                robotToGoalVector.getMagnitude())) * 2 && flywheelUpToSpeed();
    }
}
