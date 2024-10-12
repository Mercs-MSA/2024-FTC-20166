package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * Testing autonomous using Pedro based on curved back and forth example
 *
 */
@Config
@Autonomous
public class Autotest20166 extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 20;

    private enum AUTON_STATE {AUTON_START_STATE, CLOSE_GRIPPER_STATE, LIFT_ELEVATOR_STATE, LOWER_ATTACH_ELEVATOR_SUBMERSIBLE, GO_TO_SPIKES_STATE, LOWER_ELEVATOR_STATE, OPEN_GRIPPER_STATE, GO_TO_SAMPLE_ONE_STATE, PICK_UP_SAMPLE_ONE_STATE, SPIKE_ONE_TO_DROP_OFF, DROP_SAMPLE_STATE, GRAB_SECOND_SAMPLE_STATE, LIFT_ELEVATOR_TO_POSITION_STATE, GO_TO_SUBMERSIBLE_ZONE_STATE, WAIT_DONE_STATE, WAIT_PATH_DONE_STATE, MOVE_SPIKE_ONE}
    private Follower follower;

    private Path startToSubmersible;
    private Path subermersibleToSpike1;

    private Path spike1ToDropOff;
    private Path forwards;

    private AUTON_STATE currentAutonomousState = AUTON_STATE.AUTON_START_STATE;
private AUTON_STATE nextAutonomousState = AUTON_STATE.AUTON_START_STATE;
    private Point startPoint = new Point(0, 0, Point.CARTESIAN);
    private Point submersibleDropPoint = new Point(32.1996 - 8, 18.5629, Point.CARTESIAN);
    private Point spikeOneMidPoint = new Point(11.2458, -7.4492, Point.CARTESIAN);
    private Point spikeOne = new Point(28.948, -32.5841, Point.CARTESIAN);
    private Point spikeOneBackOffOne = new Point(22.9362, -14.8503, Point.CARTESIAN);
    private Point spikeOneBackOffTwo = new Point(15.6673, -39.0241, Point.CARTESIAN);
    private Point dropOff = new Point(4.7699, -42.1719, Point.CARTESIAN);
    private Point prePickupSpecimenPoint = new Point(13.3244, -32.2717, Point.CARTESIAN);
    private Point pickUpSpecimen = new Point(-0.0601, -30.6497, Point.CARTESIAN);
    private Point preSubmersibleDropPoint2 = new Point(17.5056, -11.5702, Point.CARTESIAN);
    private Point submersibleDropPoint2 = new Point(30.6377, 16.256, Point.CARTESIAN);
    private Point spikeTwoMidPoint = new Point(11.2458, -7.4492, Point.CARTESIAN);
    private Point spikeTwo = new Point(27.4778, -41.1026, Point.CARTESIAN);
    private Point dropOffTwo = new Point(4.7699, -42.1719, Point.CARTESIAN);
    private Point prePickupSpecimenPointTwo = new Point(13.3244, -32.2717, Point.CARTESIAN);
    private Point pickUpSpecimenTwo = new Point(-0.0601, -30.6497, Point.CARTESIAN);
    private Point preSubmersibleDropPoint3 = new Point(17.5056, -11.5702, Point.CARTESIAN);
    private Point submersibleDropPoint3 = new Point(30.4815,11.9667, Point.CARTESIAN);
    private Point spikeThreeMidPoint = new Point(11.2458, -7.4492, Point.CARTESIAN);
    private Point spikeThree = new Point(28.8115, -46.4973, Point.CARTESIAN);
    private Point dropOffThree = new Point(4.7699, -42.171, Point.CARTESIAN);
    private Point prePickupSpecimenPointThree = new Point(13.3244, -32.2717, Point.CARTESIAN);
    private Point pickUpSpecimenThree = new Point(-0.0601, -30.6497, Point.CARTESIAN);
    private Point preSubmersibleDropPoint4 = new Point(17.5056, -11.5702, Point.CARTESIAN);
    private Point submersibleDropPoint4 = new Point(31.3586, 8.8549, Point.CARTESIAN);



    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        forwards = new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(Math.abs(DISTANCE),0, Point.CARTESIAN), new Point(Math.abs(DISTANCE),DISTANCE, Point.CARTESIAN)));
//        backwards = new Path(new BezierCurve(new Point(Math.abs(DISTANCE),DISTANCE, Point.CARTESIAN), new Point(Math.abs(DISTANCE),0, Point.CARTESIAN), new Point(0,0, Point.CARTESIAN)));


        startToSubmersible = new Path(new BezierCurve(startPoint, submersibleDropPoint));

        subermersibleToSpike1 = new Path(new BezierCurve(submersibleDropPoint, spikeOneMidPoint, spikeOne));

        spike1ToDropOff = new Path(new BezierCurve(spikeOne, spikeOneBackOffOne, spikeOneBackOffTwo, dropOff));


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */


    //Close gripper
    //Lift elevator
    //Go to submersible
    //Attach (lower elevator) specimen onto high rung
    //Open claw
    //Go to spikes
    //Lower elevator
    //Open gripper
    //Go to sample 1
    //Pick up sample 1
    //Go to observation
    //Drop sample
    //Grab sample 2 in the observation area
    //Lift elevator to correct position
    //Go to submersible
    //Lower elevator down so the specimen will hang on the high rung.
    //Repeat until tele-op starts

    private void processStateStart()
    {
        follower.followPath(startToSubmersible);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        nextAutonomousState = AUTON_STATE.MOVE_SPIKE_ONE;
    }

    private void processWaitPathDone()
    {
        if (!follower.isBusy())
            currentAutonomousState = nextAutonomousState;
    }
    private void processWaitDoneState()
    {

    }
    private void processMoveSpikeOne()
    {
        follower.followPath(subermersibleToSpike1);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        nextAutonomousState = AUTON_STATE.SPIKE_ONE_TO_DROP_OFF;
    }

    private void processSpikeOneToDropOff()
    {
        follower.followPath(spike1ToDropOff);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        nextAutonomousState = AUTON_STATE.WAIT_DONE_STATE;
    }
    private void processStateMachine()
    {
            switch (currentAutonomousState)
            {
                case AUTON_START_STATE:
                    processStateStart();
                    break;
                case WAIT_PATH_DONE_STATE:
                    processWaitPathDone();
                    break;
                case MOVE_SPIKE_ONE:
                    processMoveSpikeOne();
                    break;
                case SPIKE_ONE_TO_DROP_OFF:
                    processSpikeOneToDropOff();
//                    break;
//                case STATE_CLOSE_AND_TURN:
//                    processStateCloseAndTurn();
//                    break;
//                case STATE_WAIT_RELEASE:
//                    processStateWaitRelease();
//                    break;
                case WAIT_DONE_STATE:
                    processWaitDoneState();
                    break;


        }

    }
    public void loop() {
        processStateMachine();
        follower.update();

//        if (!follower.isBusy()) {
//            if (forward) {
//                forward = false;
//                follower.followPath(backwards);
//            } else {
//                forward = true;
//                follower.followPath(forwards);
//            }
//        }

//        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
