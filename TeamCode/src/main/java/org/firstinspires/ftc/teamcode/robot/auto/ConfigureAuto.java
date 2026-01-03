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
    private final String[] groupNames = {"starting", "shooting", "spike mark", "gate", "human player", "wait"};
    private final String[] startingNames = {"start near","start far"};
    private final String[] shootingNames = {"shoot close","shoot middle","shoot far"};
    private final String[] spikeMarkNames = {"spike mark 1","spike mark 2","spike mark 3"};
    private final String[] gateCollectNames = {"gate fast collect", "gate friendly collect"};
    private final String[] humanPlayerCollectNames = {"human player collect"};
    private final String[] waitNames = {"wait (1 second)"};
    private final String[][] allTaskNames = {startingNames, shootingNames, spikeMarkNames, gateCollectNames, humanPlayerCollectNames, waitNames};

    private int selectedGroup = 0;
    private int selectedTaskIndex = 0;
    private boolean dpadOns = false;
    private boolean manageTaskOns = false;
    private boolean isStartSelected = false;

    @Override
    public void init() {
        routine = new ArrayList<>();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Configure Auto \ncross to add task \ncircle to delete task " +
                "\ntriangle to clear all tasks \ndpad up/down to cycle groups \ndpad left/right to cycle tasks\n");

        if (!dpadOns && isStartSelected) {
            if (gamepad1.dpad_up && selectedGroup != 5) {
                selectedGroup = Math.floorMod((selectedGroup + 1), groupNames.length);
                selectedTaskIndex = selectedGroup * 10;
            } else if (gamepad1.dpad_up) {
                selectedGroup = 1;
                selectedTaskIndex = selectedGroup * 10;
            } else if (gamepad1.dpad_down && selectedGroup != 1) {
                selectedGroup = Math.floorMod((selectedGroup - 1), groupNames.length);
                selectedTaskIndex = selectedGroup * 10;
            } else if (gamepad1.dpad_down) {
                selectedGroup = 5;
                selectedTaskIndex = selectedGroup * 10;
            }
        }

        if (gamepad1.dpad_right && !dpadOns) {
            selectedTaskIndex = Math.floorMod((selectedTaskIndex + 1) - selectedGroup * 10, allTaskNames[selectedGroup].length) + selectedGroup * 10;
        } else if (gamepad1.dpad_left && !dpadOns) {
            selectedTaskIndex = Math.floorMod((selectedTaskIndex - 1) - selectedGroup * 10, allTaskNames[selectedGroup].length) + selectedGroup * 10;
        }

        if(!manageTaskOns) {
            if (gamepad1.cross) {
                routine.add(selectedTaskIndex);
                if (!isStartSelected) {
                    isStartSelected = true;
                }
                if (routine.get(routine.size() - 1) != 50) {
                    if (routine.get(routine.size() - 1) >= 20 || routine.get(routine.size() - 1) < 10) {
                        selectedGroup = 1;
                    } else {
                        selectedGroup = 2;
                    }
                    selectedTaskIndex = selectedGroup * 10;
                }
            } else if (gamepad1.circle && routine.size() > 1) {
                routine.remove(routine.size() - 1);
            } else if (gamepad1.triangle) {
                selectedGroup = 0;
                routine.clear();
                isStartSelected = false;
                selectedTaskIndex = selectedGroup * 10;
            }
        }

        telemetry.addLine("select a " + groupNames[selectedGroup] + " task");

        telemetry.addLine("selected task = " + allTaskNames[selectedGroup][selectedTaskIndex - selectedGroup * 10]);

        telemetry.addLine("\nroutine:");
        for (int i = 0; i < routine.size(); i ++) {
            telemetry.addLine((i) + ". " + allTaskNames[(Math.floorDiv(routine.get(i), 10))]
                    [routine.get(i) - ((Math.floorDiv(routine.get(i), 10)) * 10)]);

        }

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
