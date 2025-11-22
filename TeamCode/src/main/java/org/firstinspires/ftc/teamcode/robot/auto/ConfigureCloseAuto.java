package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.List;

@Autonomous(name = "Configure Close Auto", group = "close auto")
public class ConfigureCloseAuto extends OpMode {
    private List<Integer> routine;
    private final String[] names = {"spike mark 1", "spike mark 2", "spike mark 3", "gate", "wait (0.5 seconds)"};
    private final double[] times = {4, 4.5, 5, 5, 0.5};

    private int selectedTask = 0;
    private boolean dpadOns = false;
    private boolean manageTaskOns = false;

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {
        telemetry.addLine("Configure Close Auto \ncross to add task \ncircle to delete task " +
                "\ntriangle to delete all tasks \n");

        selectedTask += (gamepad1.dpad_up && !dpadOns ? 1 : 0) - (gamepad1.dpad_down && !dpadOns ? 1 : 0);

        if(selectedTask > names.length - 1)
            selectedTask = 0;
        else if(selectedTask < 0)
            selectedTask = names.length - 1;

        telemetry.addLine("next task = " + names[selectedTask]);

        if(!manageTaskOns) {
            if (gamepad1.cross) {
                routine.add(selectedTask);
            } else if (gamepad1.circle && !routine.isEmpty()) {
                routine.remove(routine.size() - 1);
            } else if (gamepad1.triangle) {
                routine.clear();
            }
        }

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

        File file = AppUtil.getInstance().getSettingsFile("CloseConfig.txt");
        ReadWriteFile.writeFile(file, routine.toString());
    }
}
