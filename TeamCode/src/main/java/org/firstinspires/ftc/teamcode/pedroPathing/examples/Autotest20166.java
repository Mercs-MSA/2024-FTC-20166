package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pathDescriptor;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.wayPoints;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPose;
/*
import static org.firstinspires.ftc.teamcode.wayPoints.dropOffToSpecimenPickup;
import static org.firstinspires.ftc.teamcode.wayPoints.dropOffToSpecimenPickupHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.specimenPickUpToSubmersible2;
import static org.firstinspires.ftc.teamcode.wayPoints.specimenPickUpToSubmersible2Heading;
import static org.firstinspires.ftc.teamcode.wayPoints.spike1ToDropOff;
import static org.firstinspires.ftc.teamcode.wayPoints.spike1ToDropOffHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.startToSubmersible;
import static org.firstinspires.ftc.teamcode.wayPoints.startToSubmersibleHeading;
import static org.firstinspires.ftc.teamcode.wayPoints.submersibleToSpike1;
import static org.firstinspires.ftc.teamcode.wayPoints.submersibleToSpike1Heading;
//import static org.firstinspires.ftc.teamcode.wayPoints.*;
*/
/**
 * Testing autonomous using Pedro based on curved back and forth example
 *
 */
@Config
@Autonomous
public class Autotest20166 extends OpMode {
    private Telemetry telemetryA;

 //   private wayPoints myWayPoints = new wayPoints();

    public double botHeading = startingPose.getHeading();

    private enum AUTON_STATE {AUTON_START_STATE, CLOSE_GRIPPER_STATE, LIFT_ELEVATOR_STATE, LOWER_ATTACH_ELEVATOR_SUBMERSIBLE, GO_TO_SPIKES_STATE, LOWER_ELEVATOR_STATE, OPEN_GRIPPER_STATE, GO_TO_SAMPLE_ONE_STATE, PICK_UP_SAMPLE_ONE_STATE, SPIKE_ONE_TO_DROP_OFF, DROP_SAMPLE_STATE, GRAB_SECOND_SAMPLE_STATE, LIFT_ELEVATOR_TO_POSITION_STATE, GO_TO_SUBMERSIBLE_ZONE_STATE, WAIT_DONE_STATE, WAIT_PATH_DONE_STATE, MOVE_SPIKE_ONE}
    private Follower follower;

    private AUTON_STATE currentAutonomousState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE nextAutonomousState = AUTON_STATE.AUTON_START_STATE;

    public static final Point startPoint = new Point (startingPose.getX(), startingPose.getY(), Point.CARTESIAN);
    public static final Point submersibleDropPoint = new Point(4.5, -50, Point.CARTESIAN); //Heading 1.542 (90)
    public static final Point spikeOneMidPoint = new Point(13, -49.4, Point.CARTESIAN); //Heading 0.811 (45)
    public static final Point spikeOne = new Point(53, -37.2, Point.CARTESIAN); //Heading 1.493 (90)
    public static final Point spikeOneBackOffOne = new Point(52.3, -44.7, Point.CARTESIAN); //Heading 1.493 (90)
    public static final Point dropOff = new Point(57.4, -57.6, Point.CARTESIAN); //Heading 4.659 (266)
    public static final double dropOffHeading = Math.toRadians(266);
    public static final Point pickupSpecimenMid = new Point(34.9, -58.3, Point.CARTESIAN); //Heading 4.669 (266)
    public static final Point pickUpSpecimen = new Point(34.6, -60, Point.CARTESIAN); //Heading 4.648 (268)
    public static final Point submersibleDropPoint2 = new Point(5, -29.4, Point.CARTESIAN); //Heading 1.454 (83)
    public static final Point spikeTwoMidPoint = new Point(38.1, -51.3, Point.CARTESIAN); //Heading 1.031 (60)
    public static final Point spikeTwo = new Point(61.5, -37.4, Point.CARTESIAN); //Heading 1.481 (90)
    public static final Point dropOffTwo = new Point(49.8, -60.7, Point.CARTESIAN); //Heading 4.751 (268)
    public static  final Point prePickupSpecimenPointTwo = new Point(43.9, -53.9, Point.CARTESIAN); //Heading 4.701 (270)
    public static  final Point pickUpSpecimenTwo = new Point(34.6, -62.9, Point.CARTESIAN); //Heading 4.648 (266)
    //private final Point preSubmersibleDropPoint3 = new Point(17.5056, -11.5702, Point.CARTESIAN);
    public static  final Point submersibleDropPoint3 = new Point(2.1,-31.4, Point.CARTESIAN); //Heading 1.511 (90)
    public static  final Point spikeThreeMidPoint = new Point(38.8, -49.3, Point.CARTESIAN); //Heading 0.747 (42)
    public static  final Point spikeThree = new Point(62.5, -29.6, Point.CARTESIAN); //Heading 6.213 (355)
    public static  final Point dropOffThree = new Point(49.8, -60.7, Point.CARTESIAN); //Heading 4.751 (272)
    public static  final Point prePickupSpecimenPointThree = new Point(43.9, -53.9, Point.CARTESIAN); //Heading 4.701 (270)
    public static  final Point pickUpSpecimenThree = new Point(34.6, -62.9, Point.CARTESIAN); //Heading 4.648 (270)
    //private final Point preSubmersibleDropPoint4 = new Point(17.5056, -11.5702, Point.CARTESIAN);
    public static  final Point submersibleDropPoint4 = new Point(2.4, -28.2, Point.CARTESIAN); //Heading 1.473 (90)
    public static  final Point pickUpSpecimenFour = new Point(34.6, -62.9, Point.CARTESIAN); //Heading 4.648 (266)

    public static  final Point submersibleDropPoint5 = new Point(-0.4, -29, Point.CARTESIAN); //Heading 1.485 (90)
    public static  final Point prePark = new Point (16.4, -38.9, Point.CARTESIAN); //Heading 1.972 (112)
    public static  final Point park = new Point(18.3, -30.4, Point.CARTESIAN); //Heading 2.06 (118)

    //Paths
    public static  final Path startToSubmersible = new Path(new BezierCurve(startPoint, submersibleDropPoint));
    public static final double startToSubmersibleHeading = Math.toRadians(90);

    public static  final Path submersibleToSpike1 = new Path(new BezierCurve(submersibleDropPoint, spikeOneMidPoint, spikeOne));
    public static final double submersibleToSpike1Heading = Math.toRadians(90);

    public static  final Path spike1ToDropOff = new Path(new BezierCurve(spikeOne, spikeOneBackOffOne, dropOff));
    public static final double spike1ToDropOffHeading = 90;
    public static  final Path dropOffToSpecimenPickup = new Path(new BezierCurve(dropOff, pickupSpecimenMid, pickUpSpecimen));
    public static double dropOffToSpecimenPickupHeading = Math.toRadians(266);

    public static final Path specimenPickUpToSubmersible2 = new Path(new BezierCurve(pickUpSpecimen, submersibleDropPoint2));
    public static final double specimenPickUpToSubmersible2Heading = Math.toRadians(90);
    public static final pathDescriptor startToSubmersibleSet = new pathDescriptor(startToSubmersible, startToSubmersibleHeading);
    public static final pathDescriptor specimenPickUpToSubmersible2Set = new pathDescriptor(specimenPickUpToSubmersible2, specimenPickUpToSubmersible2Heading);
    public static final pathDescriptor dropOffToSpecimenPickUpSet = new pathDescriptor(dropOffToSpecimenPickup, dropOffToSpecimenPickupHeading);
    public static final pathDescriptor spike1ToDropOffSet = new pathDescriptor(spike1ToDropOff, spike1ToDropOffHeading);
    public static final pathDescriptor submersibleToSpike1Set = new pathDescriptor(submersibleToSpike1, submersibleToSpike1Heading);

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override

    public void init() {
        follower = new Follower(hardwareMap);
        sleep(1000);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(0.7);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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

    private void setupPath(Path pathToFollow, double endHeading)
    {
        double currentHeading = botHeading;
        //Path toFollowPath = startToSubmersibleSet.pathToFollow;
        //Path toFollowPath = pathSetToFollow.pathToFollow;
        //double endHeading = startToSubmersibleHeading;
        //toFollowPath.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        follower.followPath(pathToFollow);
        pathToFollow.setLinearHeadingInterpolation(currentHeading, endHeading);
        botHeading = endHeading;
    }

    private void processStateStart()
    {
        setupPath(startToSubmersible, startToSubmersibleHeading);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        nextAutonomousState = AUTON_STATE.MOVE_SPIKE_ONE;
        //nextAutonomousState = AUTON_STATE.WAIT_DONE_STATE;
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
        //pathDescriptor newPath = new pathDescriptor(startToSubmersible, startToSubmersibleHeading);
        setupPath(submersibleToSpike1, submersibleToSpike1Heading);
        //setupPath(newPath);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        nextAutonomousState = AUTON_STATE.SPIKE_ONE_TO_DROP_OFF;
    }

    private void processSpikeOneToDropOff()
    {
        setupPath(spike1ToDropOff, spike1ToDropOffHeading);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        nextAutonomousState = AUTON_STATE.GRAB_SECOND_SAMPLE_STATE;
    }
    private void processDropOffToSpecimenPickUp()
    {
        setupPath(dropOffToSpecimenPickup, dropOffToSpecimenPickupHeading);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        nextAutonomousState = AUTON_STATE.WAIT_DONE_STATE;
    }
    private void processSpecimenPickUpToSubmersible()
    {
        setupPath(specimenPickUpToSubmersible2, specimenPickUpToSubmersible2Heading);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
    }

    private void processStateMachine()
    {
            switch (currentAutonomousState)
            {
                case AUTON_START_STATE:
                    //pathDescriptor test = new pathDescriptor(startToSubmersible, startToSubmersibleHeading);
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
                    processDropOffToSpecimenPickUp();
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
