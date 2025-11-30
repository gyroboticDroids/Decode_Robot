package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "configure auto", group = "far auto")
public class ConfigureAuto extends OpMode {
    private List<Integer> routine;
    private final String[] names = {"spike mark 1", "spike mark 2", "spike mark 3", "gate", "human player", "wait (1 second)"};
    private final String[] scoreNames = {"score close","score middle","score far"};
    private final double[] times = {4, 4.5, 5, 5, 5, 1};

    private int selectedTask = 0;
    private int selectedScorePosition = 0;
    private int selectedStartScorePosition = 0;
    private boolean dpadOns = false;
    private boolean startOns = false;
    private boolean manageTaskOns = false;

    @Override
    public void init() {
        routine = new ArrayList<>();
        routine.add(0);
        routine.add(0);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Configure Far Auto \ncross to add task \ncircle to delete task " +
                "\ntriangle to delete all tasks \nsquare to toggle shooting area");

        selectedTask += (gamepad1.dpad_up && !dpadOns ? 1 : 0) - (gamepad1.dpad_down && !dpadOns ? 1 : 0);
        selectedScorePosition += (gamepad1.dpad_right && !dpadOns ? 1 : 0) - (gamepad1.dpad_left && !dpadOns ? 1 : 0);
        selectedStartScorePosition += (gamepad1.right_bumper && !startOns ? 1 : 0) - (gamepad1.left_bumper && !startOns ? 1 : 0);

        selectedTask = Math.floorMod(selectedTask, names.length);
        selectedScorePosition = Math.floorMod(selectedScorePosition, 3);
        selectedStartScorePosition = Math.floorMod(selectedStartScorePosition, 3);

        telemetry.addLine("next task = " + names[selectedTask]);
        telemetry.addLine("score position = " + scoreNames[selectedScorePosition]);

        if(!manageTaskOns) {
            if (gamepad1.cross) {
                routine.add(selectedTask);
                routine.add(selectedScorePosition);
            } else if (gamepad1.circle && routine.size() > 1) {
                routine.remove(routine.size() - 1);
                routine.remove(routine.size() - 1);
            } else if (gamepad1.triangle) {
                routine.subList(2, routine.size()).clear();
            }
        }

        if(gamepad1.square && !startOns) {
            if(routine.get(0) == 0) {
                routine.set(0, 1);
                selectedStartScorePosition = 2;
            } else {
                routine.set(0, 0);
                selectedStartScorePosition = 1;
            }
        }

        routine.set(1, selectedStartScorePosition);

        startOns = gamepad1.square || gamepad1.left_bumper || gamepad1.right_bumper;

        telemetry.addLine("starting " + ((routine.get(0) == 0)? "far" : "close"));
        telemetry.addLine("preload " + scoreNames[routine.get(1)]);

        telemetry.addLine("routine:");
        for (int i = 3; i < routine.size(); i += 2) {
            telemetry.addLine(i / 2 - 0.5 + ". " + names[routine.get(i)] + ", " + scoreNames[routine.get(i + 1)]);
        }

        double autoTime = 0;
        for (int i = 0; i < routine.size(); i += 2) {
            autoTime += times[routine.get(i)];
        }

        telemetry.addLine("\nauto run time = " + autoTime);

        telemetry.update();

        dpadOns = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right;
        manageTaskOns = gamepad1.cross || gamepad1.circle || gamepad1.triangle;
    }

    public void loop() {
        if (routine.isEmpty()) {
            requestOpModeStop();
        }

        String routineString = routine.toString();
        routineString = routineString.substring(1, routineString.length() - 1);

        File file = AppUtil.getInstance().getSettingsFile("Config.txt");
        ReadWriteFile.writeFile(file, routineString);

        requestOpModeStop();
    }
}
