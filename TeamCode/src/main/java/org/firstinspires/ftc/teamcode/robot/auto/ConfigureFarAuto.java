package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "configure far auto", group = "far auto")
public class ConfigureFarAuto extends OpMode {
    private List<Integer> routine;
    private final String[] names = {"spike mark 1", "spike mark 2", "spike mark 3", "gate", "human player", "wait (1 second)"};
    private final double[] times = {4, 4.5, 5, 5, 1};

    private int selectedTask = 0;
    private boolean dpadOns = false;
    private boolean prevGpadSquare = false;
    private boolean manageTaskOns = false;

    @Override
    public void init() {
        routine = new ArrayList<>();
        routine.add(0);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Configure Far Auto \ncross to add task \ncircle to delete task " +
                "\ntriangle to delete all tasks \nsquare to toggle shooting area");

        selectedTask += (gamepad1.dpad_up && !dpadOns ? 1 : 0) - (gamepad1.dpad_down && !dpadOns ? 1 : 0);

        if(selectedTask > names.length - 1)
            selectedTask = 0;
        else if(selectedTask < 0)
            selectedTask = names.length - 1;

        telemetry.addLine("next task = " + names[selectedTask]);

        if(!manageTaskOns) {
            if (gamepad1.cross) {
                routine.add(selectedTask);
            } else if (gamepad1.circle && routine.size() > 1) {
                routine.remove(routine.size() - 1);
            } else if (gamepad1.triangle) {
                routine.subList(1, routine.size()).clear();
            }
        }

        if(gamepad1.square && !prevGpadSquare) {
            if(routine.get(0) == 0) {
                routine.set(0, 1);
            } else {
                routine.set(0, 0);
            }
        }

        prevGpadSquare = gamepad1.square;

        telemetry.addLine("scoring " + ((routine.get(0) == 0)? "far" : "close"));

        telemetry.addLine("routine:");
        for (int i = 0; i < routine.size(); i++) {
            telemetry.addLine(i + 1 + ". " + names[routine.get(i)]);
        }

        double autoTime = 0;
        for (int task : routine) {
            autoTime += times[task];
        }

        telemetry.addLine("\nauto run time = " + autoTime);

        telemetry.update();

        dpadOns = gamepad1.dpad_up || gamepad1.dpad_down;
        manageTaskOns = gamepad1.cross || gamepad1.circle || gamepad1.triangle;
    }

    public void loop() {
        if (routine.isEmpty()) {
            requestOpModeStop();
        }

        String routineString = routine.toString();
        routineString = routineString.substring(1, routineString.length() - 1);

        File file = AppUtil.getInstance().getSettingsFile("CloseConfig.txt");
        ReadWriteFile.writeFile(file, routineString);

        requestOpModeStop();
    }
}
