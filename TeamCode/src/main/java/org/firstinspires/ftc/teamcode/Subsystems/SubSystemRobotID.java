package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubSystemRobotID {
    private int robotID = 0;

    private DigitalChannel limitSwitch;
    private DigitalChannel limitSwitchTwo;
    public SubSystemRobotID(HardwareMap hardwareMap) throws InterruptedException {
        limitSwitch = hardwareMap.get(DigitalChannel.class, "limitSwitch");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        limitSwitchTwo = hardwareMap.get(DigitalChannel.class, "limitSwitchTwo");
        limitSwitchTwo.setMode(DigitalChannel.Mode.INPUT);

    }
    public int getRobotID()
    {
        if(limitSwitch.getState())
        {
            robotID = 0;
        }
        else if (limitSwitchTwo.getState())
        {
            robotID = 2;
        }
        else
        {
            robotID = 1;
        }
        return robotID;
    }

    public int getRobotDigital(){
        int status = 0;
        if (limitSwitch.getState())
            status = status + 1;
        if (limitSwitchTwo.getState())
            status = status + 2;
        return status;
    }

    }
