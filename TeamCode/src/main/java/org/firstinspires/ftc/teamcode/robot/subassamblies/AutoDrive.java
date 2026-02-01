package org.firstinspires.ftc.teamcode.robot.subassamblies;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class AutoDrive {
    private final Drive drive;
    private final Intake intake;
    private final Shooter shooter;
    private final Hardware hardware;

    private int state = -1;
    private boolean isBusy = false;

    public AutoDrive(Drive d, Intake i, Shooter s, Hardware h) {
        drive = d;
        intake = i;
        shooter = s;
        hardware = h;
    }

    public void update() {
        drive.setAutoDriveIsActive(isBusy);

        switch (state) {
            case 0:
                Path gateBump = new Path(new BezierLine(hardware.follower.getPose(), new Pose(115, 63, Math.toRadians(0))));
                gateBump.setLinearHeadingInterpolation(hardware.follower.getHeading(), 0);
                hardware.follower.followPath(gateBump);
                state++;
                break;

            case 1:
                if(hardware.follower.getCurrentTValue() > 0.97) {
                    Path gateCollect = new Path(new BezierLine(new Pose(115, 63, Math.toRadians(0)), new Pose(130, 56, Math.toRadians(36))));
                    gateCollect.setLinearHeadingInterpolation(0, Math.toRadians(36));
                    hardware.follower.followPath(gateCollect);
                    state++;
                }
                break;
        }
    }

    public void driveToGate() {
        state = 0;
        isBusy = true;
    }

    public boolean isBusy() {
        return isBusy;
    }

    public void cancel() {
        state = -1;
        isBusy = false;
    }
}
