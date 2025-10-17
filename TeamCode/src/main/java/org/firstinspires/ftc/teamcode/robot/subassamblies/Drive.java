package org.firstinspires.ftc.teamcode.robot.subassamblies;


import com.qualcomm.robotcore.hardware.Gamepad;

public class Drive {
    Hardware hardware;
    Gamepad gamepad;

    public Drive(Hardware hardware, Gamepad gamepad){
        this.hardware = hardware;
        this.gamepad = gamepad;
    }
}
