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
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPose;
import static org.firstinspires.ftc.teamcode.wayPoints.*;



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
    private Path dropoffToSpecimenPickup;

    private AUTON_STATE currentAutonomousState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE nextAutonomousState = AUTON_STATE.AUTON_START_STATE;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(0.7);

        startToSubmersible = new Path(new BezierCurve(startPoint, submersibleDropPoint));
        startToSubmersible.setLinearHeadingInterpolation(startingPose.getHeading(), submersibleDropPointHeading);

        subermersibleToSpike1 = new Path(new BezierCurve(submersibleDropPoint, spikeOneMidPoint, spikeOne));
        subermersibleToSpike1.setLinearHeadingInterpolation(submersibleDropPointHeading, spikeOneHeading);

        spike1ToDropOff = new Path(new BezierCurve(spikeOne, spikeOneBackOffOne, dropOff));
        spike1ToDropOff.setLinearHeadingInterpolation(spikeOneHeading, dropOffHeading);

        dropoffToSpecimenPickup = new Path(new BezierCurve(dropOff, pickupSpecimenMid, pickUpSpecimen));
        dropoffToSpecimenPickup.setLinearHeadingInterpolation(dropOffHeading, pickUpSpecimenHeading);

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
        nextAutonomousState = AUTON_STATE.GRAB_SECOND_SAMPLE_STATE;
    }
    private void processGrabSecondSampleState()
    {
        follower.followPath(dropoffToSpecimenPickup);
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
                    break;
                case GRAB_SECOND_SAMPLE_STATE:
                    processGrabSecondSampleState();
                    break;
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

        follower.telemetryDebug(telemetryA);
    }
}
