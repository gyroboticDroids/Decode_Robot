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
    private final String[] groupNames = {"starting", "shooting", "spike mark", "gate collect", "human player collect", "wait (1 second)"};
    private final String[] startingNames = {"start near","start far"};
    private final String[] shootingNames = {"shoot close","shoot middle","shoot far"};
    private final String[] spikeMarkNames = {"spike mark 1","spike mark 2","spike mark 3"};
    private final String[] gateCollectNames = {"gate"};
    private final String[] humanPlayerCollectNames = {"human player"};
    private final String[] waitNames = {"wait"};
    private final String[][] allTaskNames = {startingNames, shootingNames, spikeMarkNames, gateCollectNames, humanPlayerCollectNames, waitNames};
    private final double[] times = {4, 4.5, 5, 5, 5, 1};

    private int selectedGroup = 0;
    private int selectedTaskValue = 0;
    private boolean dpadOns = false;
    private boolean manageTaskOns = false;

    @Override
    public void init() {
        routine = new ArrayList<>();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Configure Auto \ncross to add task \ncircle to delete task " +
                "\ntriangle to clear all tasks \n");


        if(gamepad1.dpad_up && !dpadOns){
            selectedGroup = (selectedGroup + 1) % groupNames.length;
            selectedTaskValue = selectedGroup * 10;
        } else if(gamepad1.dpad_down && !dpadOns){
            selectedGroup = (selectedGroup - 1 + groupNames.length) % groupNames.length;
            selectedTaskValue = selectedGroup * 10;
        }

        if (gamepad1.dpad_right && !dpadOns) {
            selectedTaskValue = Math.floorMod((selectedTaskValue + 1) - selectedGroup * 10, allTaskNames[selectedGroup].length) + selectedGroup * 10;
        } else if (gamepad1.dpad_left && !dpadOns) {
            selectedTaskValue = Math.floorMod((selectedTaskValue - 1) - selectedGroup * 10, allTaskNames[selectedGroup].length) + selectedGroup * 10;
        }

        if(!manageTaskOns) {
            if (gamepad1.cross) {
                routine.add(selectedTaskValue);
            } else if (gamepad1.circle && routine.size() > 1) {
                routine.remove(routine.size() - 1);
            } else if (gamepad1.triangle) {
                routine.clear();
            }
        }

        telemetry.addLine("Selected Task = " + allTaskNames[selectedGroup][selectedTaskValue - selectedGroup * 10]);

        telemetry.addLine("Selected Group = " + groupNames[selectedGroup]);

        double autoTime = 0;
        telemetry.addLine("\nroutine:");
        for (int i = 0; i < routine.size(); i ++) {
            telemetry.addLine((i) + ". " + groupNames[Math.floorDiv(routine.get(i), 10)] + ": " +
                    allTaskNames[(Math.floorDiv(routine.get(i), 10))][routine.get(i) - ((Math.floorDiv(routine.get(i), 10)) * 10)]);
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
