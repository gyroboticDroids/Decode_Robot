package org.firstinspires.ftc.teamcode.robot.teaching;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name = "save to file")
public class SaveToFile extends OpMode {
    private List<Integer> routine;

    @Override
    public void init() {
        routine = new ArrayList<>();
    }

    @Override
    public void init_loop() {
        if(gamepad1.circleWasReleased()) {
            routine.add(0);
        } else if(gamepad1.squareWasReleased()) {
            routine.add(4);
        } else if(gamepad1.crossWasReleased()) {
            routine.add(13);
        } else if(gamepad1.triangleWasReleased()) {
            routine.add(28);
        }

        telemetry.addData("routine", routine);
    }

    @Override
    public void loop() {
        String routineString = routine.toString();
        routineString = routineString.substring(1, routineString.length() - 1);

        File file = AppUtil.getInstance().getSettingsFile("File.txt");
        ReadWriteFile.writeFile(file, routineString);

        requestOpModeStop();
    }
}
