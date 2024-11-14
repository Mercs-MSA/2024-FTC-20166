package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.robot.Robot;

@Config
public class RobotConstants
{
    public static double ELEVATOR_MULTIPLIER;//Bot A = 1.0, Bot B = -1.0
    public static final int ELEVATOR_BOTTOM_POSITION = 0;//Lowest position, on the floor
    public static final int ELEVATOR_TOP_RUNG_PLACE = -2100;//Upper rung starting position

    public static final int ELEVATOR_TOP_RUNG_RELEASE = -1450;//Upper rung pull down position
    public static final int ELEVATOR_TOP_BASKET = -3900;//Top basket location
    public static final int ELEVATOR_LOW_BASKET = -2550;//Low basket location
    public static final int ELEVATOR_TRANSFER_SAMPLE_POSITION = -705;//Low basket location


    //   private static final int ELEVATOR_BOTTOM_RUNG_PLACE = -400;

    //   private static final int ELEVATOR_BOTTOM_RUNG_RELEASE = -225;

    //   private static final int ELEVATOR_TOP_BASKET = -1401;

    //   private static final int ELEVATOR_MIDDLE_BASKET = -896;

    public static final int ELEVATOR_SPECIMEN_PICK_UP_LIFT_POSITION = -1100;
    public static final int ELEVATOR_SPECIMEN_PICKUP = -830;
    public static final double GRABBER_OPEN_POSITION = 0.7;
    public static final double GRABBER_CLOSE_POSITION = 0.95;
    public static double INTAKE_ARM_TRANSFER_SAMPLE_POSITION = 0.23;
    public static double INTAKE_ARM_UP_POSITION = 0.3;
    public static double INTAKE_ARM_DOWN_POSITION = 0.75;

    public static final double START_TO_SUBMERSIBLE_SPEED = 0.8;
    public static final double SUBMERSIBLE_TO_PUSH_SPEED = 0.9;

    public static final double INTAKE_ROLLER_OUT_SPEED = 1;
    public static final double INTAKE_ROLLER_IN_SPEED = -1;
    public static final double INTAKE_ROLLER_HOLD_SPEED = -0.2;
    public static final double STALL_THRESHOLD = 0.02;
    public static final int STALL_SAMPLE_COUNT = 10;

    public RobotConstants(int robotId) {
        if (robotId == 0) { //Bot A
            ELEVATOR_MULTIPLIER = 1;
        } else if (robotId == 1){
            ELEVATOR_MULTIPLIER = -1;
        } else {
            ELEVATOR_MULTIPLIER = 1;
        }
    }
}
