package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "configure auto", group = "auto")
public class ConfigureAuto extends OpMode {
    private List<Integer> routine;
    private final String[] names = {"starting", "shooting", "spike mark", "gate collect", "human player collect", "wait (1 second)"};
    private final String[] startingNames = {"near","far"};
    private final String[] shootingNames = {"shoot close","shoot middle","shoot far"};
    private final String[] spikeMarkNames = {"spike mark 1","spike mark 2","spike mark 3"};
    private final String[] gateCollectNames = {"gate"};
    private final String[] humanPlayerCollectNames = {"human player"};
    private final String[] waitNames = {"wait"};
    private final double[] times = {4, 4.5, 5, 5, 5, 1};

    private int selectedGroup = 0;
    private int selectedTask = 0;
    private int selectedStartScorePosition = 1;
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
        telemetry.addLine("Configure Auto \ncross to add task \ncircle to delete task " +
                "\ntriangle to delete all tasks \nsquare to toggle shooting area \n");

        selectedGroup += (gamepad1.dpad_up && !dpadOns ? 1 : 0) - (gamepad1.dpad_down && !dpadOns ? 1 : 0);
        selectedTask += (gamepad1.dpad_right && !dpadOns ? 1 : 0) - (gamepad1.dpad_left && !dpadOns ? 1 : 0);

        selectedGroup = Math.floorMod(selectedGroup, names.length);
        selectedTask = Math.floorMod(selectedTask, (selectedGroup == 0) ? startingNames.length : (selectedGroup == 1) ? shootingNames.length + 10 :
                (selectedGroup == 2) ? spikeMarkNames.length + 20 : (selectedGroup == 3) ? gateCollectNames.length + 30 :
                (selectedGroup == 4) ? humanPlayerCollectNames.length + 40 : waitNames.length + 50);

        telemetry.addLine("next group = " + names[selectedGroup]);
        telemetry.addLine("next task = " + names[selectedTask]);

        if(!manageTaskOns) {
            if (gamepad1.cross) {
                routine.add(selectedTask);
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

        telemetry.addLine("\nstarting " + ((routine.get(0) == 0)? "close" : "far"));

        telemetry.addLine("\nroutine:");
        for (int i = 2; i < routine.size() - 1; i += 2) {
            telemetry.addLine(i / 2 + ". " + names[routine.get(i)] + ", " + startingNames[routine.get(i + 1)]);
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
