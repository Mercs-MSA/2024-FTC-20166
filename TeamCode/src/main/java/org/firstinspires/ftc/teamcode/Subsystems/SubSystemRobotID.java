package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubSystemRobotID {
    private int robotID = 0;
    private DigitalChannel limitSwitch;
    public SubSystemRobotID(HardwareMap hardwareMap) throws InterruptedException {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        if(limitSwitch.getState()){
            robotID = 0;
        } else {
            robotID = 1;
        }
    }
    public int getRobotID()
    {
        return robotID;
    }


    }
