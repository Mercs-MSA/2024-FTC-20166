package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.pathDescriptor;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;

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

    private RobotConstants constants = new RobotConstants();
 //   private wayPoints myWayPoints = new wayPoints();
    private SubSystemGrabber robotGrabber;
    private SubSystemElevator robotElevator;

    public double botHeading = startingPose.getHeading();

    private enum AUTON_STATE
    {
        AUTON_START_STATE,
        LOWER_ELEVATOR_STATE,
        SPIKE_ONE_TO_DROP_OFF,
        GRAB_SECOND_SAMPLE_STATE,
        WAIT_AUTO_FINISHED,
        WAIT_PATH_DONE_STATE,
        MOVE_SPIKE_ONE,
        PUSH_SAMPLES_STATE,
    }
    private Follower follower;

    private AUTON_STATE currentAutonomousState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE waitPathDoneNextState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE lowerElevatorNextState = AUTON_STATE.AUTON_START_STATE;

    public static final Point startPoint = new Point (startingPose.getX(), startingPose.getY(), Point.CARTESIAN);
    public static final Point submersibleDropPoint = new Point(2.4, -32.6, Point.CARTESIAN); //Heading 1.542 (90)
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

    //New Points for submersible to spikes ready to push into human player area
    public static final Point submersibleToSpike1 = new Point(25.3,-52 , Point.CARTESIAN);
    public static final Point submersibleToSpike2 = new Point(33.4,-48.8 , Point.CARTESIAN);
    public static final Point submersibleToSpike3 = new Point(33.4,-15.0 , Point.CARTESIAN);
    public static final Point submersibleToSpike4 = new Point(42.5,-12.8 , Point.CARTESIAN);
    public static final Point submersibleToSpike5 = new Point(42.4,-17.8 , Point.CARTESIAN);

        //ready to push sample one

    public static final Point submersibleToSpike6 = new Point(47.5,-55.1 , Point.CARTESIAN); //deposited sample 1

    public static final Point submersibleToSpike7 = new Point(43.1,-12 , Point.CARTESIAN);
    public static final Point submersibleToSpike8 = new Point(55.3,-11.5 , Point.CARTESIAN);
    public static final Point submersibleToSpike9 = new Point(56.3,-16.7 , Point.CARTESIAN);

        //ready to push sample 2
    public static final Point submersibleToSpike10 = new Point(55,-56.1 , Point.CARTESIAN); //deposited sample 2
    public static final Point submersibleToSpike11 = new Point(54.8,-15.3 , Point.CARTESIAN);
    public static final Point submersibleToSpike12 = new Point(61.6,-15.2 , Point.CARTESIAN);
    public static final Point submersibleToSpike13 = new Point(61.6,-18 , Point.CARTESIAN);
        //ready to push sample 3
    public static final Point submersibleToSpike14 = new Point(57.8,-57.4 , Point.CARTESIAN); //deposited sample 3

    //Paths
//    public static final Path submersibleToSpikePath = new Path(new BezierCurve(submersibleToSpike1, submersibleToSpike2, submersibleToSpike3, submersibleToSpike4, submersibleToSpike5, submersibleToSpike6, submersibleToSpike7, submersibleToSpike8, submersibleToSpike9, submersibleToSpike10, submersibleToSpike11, submersibleToSpike12,submersibleToSpike13, submersibleToSpike14));
    public static final Path submersibleToSpikePathSegment1 = new Path(new BezierCurve(submersibleDropPoint, submersibleToSpike1, submersibleToSpike2));
    public static final Path submersibleToSpikePathSegment2 = new Path(new BezierCurve(submersibleToSpike2, submersibleToSpike3));
    public static final Path submersibleToSpikePathSegment3 = new Path(new BezierCurve(submersibleToSpike3, submersibleToSpike4));
    public static final Path submersibleToSpikePathSegment4 = new Path(new BezierCurve(submersibleToSpike4, submersibleToSpike5, submersibleToSpike6));
    public static final Path submersibleToSpikePathSegment5 = new Path(new BezierCurve(submersibleToSpike6, submersibleToSpike7));
    public static final Path submersibleToSpikePathSegment6 = new Path(new BezierCurve(submersibleToSpike7, submersibleToSpike8));
    public static final Path submersibleToSpikePathSegment7 = new Path(new BezierCurve(submersibleToSpike8, submersibleToSpike9));

    public static final PathChain submersibleToSpikeOneChain = new PathChain(submersibleToSpikePathSegment1, submersibleToSpikePathSegment2, submersibleToSpikePathSegment3, submersibleToSpikePathSegment4, submersibleToSpikePathSegment5, submersibleToSpikePathSegment6, submersibleToSpikePathSegment7);


    public static final double submersibleToSpikePathHeading = Math.toRadians(90);
    public static  final Path startToSubmersible = new Path(new BezierCurve(startPoint, submersibleDropPoint));
    public static final double startToSubmersibleHeading = Math.toRadians(90);

    public static  final Path submersibleToSample1 = new Path(new BezierCurve(submersibleDropPoint, spikeOneMidPoint, spikeOne));
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
    public static final pathDescriptor submersibleToSpike1Set = new pathDescriptor(submersibleToSample1, submersibleToSpike1Heading);

    public static int didItWork = 0;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init()
    {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(0.7);
        try {
            initializeSubSystems();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        initalizePathHeadings();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

    }
    private void initalizePathHeadings()
    {
        submersibleToSpikePathSegment1.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment2.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment3.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment4.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment5.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment6.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        submersibleToSpikePathSegment7.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
    }
    private void initializeSubSystems() throws InterruptedException {
        robotElevator = new SubSystemElevator(hardwareMap, constants.ELEVATOR_MULTIPLIER);

        robotGrabber = new SubSystemGrabber(hardwareMap);
        robotGrabber.setPosition(constants.GRABBER_CLOSE_POSITION);

        //robotIntake = new SubSystemIntake(hardwareMap);

        //robotIntakeArm = new SubSystemIntakeArm(hardwareMap);
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
    private void setupPathChain(PathChain pathChainToFollow, double endHeading)
    {
        double currentHeading = botHeading;
        //Path toFollowPath = startToSubmersibleSet.pathChainToFollow;
        //Path toFollowPath = pathSetToFollow.pathChainToFollow;
        //double endHeading = startToSubmersibleHeading;
        //toFollowPath.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90));
        //follower.followPath(pathChainToFollow);
        follower.followPath(pathChainToFollow);
        //submersibleToSpikePathSegment1.setLinearHeadingInterpolation(currentHeading, endHeading);
        botHeading = endHeading;
        didItWork++;
    }


    private void processStateStart()
    {
        setupPath(startToSubmersible, startToSubmersibleHeading);
        robotElevator.setPosition(constants.ELEVATOR_TOP_RUNG_PLACE);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.LOWER_ELEVATOR_STATE;
        lowerElevatorNextState = AUTON_STATE.PUSH_SAMPLES_STATE;
    }

    private void processLowerElevatorState() {
        robotElevator.setPosition(constants.ELEVATOR_TOP_RUNG_RELEASE);
        if(Math.abs(robotElevator.getPosition() - constants.ELEVATOR_TOP_RUNG_RELEASE) < 20) {
            robotGrabber.setPosition(constants.GRABBER_OPEN_POSITION);
            currentAutonomousState = lowerElevatorNextState;
            robotElevator.setPosition(constants.ELEVATOR_BOTTOM_POSITION);
            follower.setMaxPower(1.0);
        }

    }

    private void processWaitPathDone()
    {
        if (!follower.isBusy())
            currentAutonomousState = waitPathDoneNextState;
    }
    private void processWaitDoneState()
    {

    }

   private void processMoveSpikeOne()
    {
        //pathDescriptor newPath = new pathDescriptor(startToSubmersible, startToSubmersibleHeading);
        //setupPath(submersibleToSample1, submersibleToSpike1Heading);
        //setupPath(newPath);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.WAIT_AUTO_FINISHED;
    }

    private void processPushSampleToObservation()
    {
        //setupPath(submersibleToSpikePath, submersibleToSpikePathHeading);
        setupPathChain(submersibleToSpikeOneChain, submersibleToSpike1Heading);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.WAIT_AUTO_FINISHED;
    }

    private void processSpikeOneToDropOff()
    {
        setupPath(spike1ToDropOff, spike1ToDropOffHeading);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.GRAB_SECOND_SAMPLE_STATE;
    }
    private void processDropOffToSpecimenPickUp()
    {
        setupPath(dropOffToSpecimenPickup, dropOffToSpecimenPickupHeading);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.WAIT_AUTO_FINISHED;
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
                case WAIT_AUTO_FINISHED:
                    processWaitDoneState();
                    break;
                case LOWER_ELEVATOR_STATE:
                    processLowerElevatorState();
                    break;
                case PUSH_SAMPLES_STATE:
                        processPushSampleToObservation();
                        break;
        }

    }
    public void loop() {
        processStateMachine();
        follower.update();
        telemetryA.addData("Did it work? ", didItWork);
        telemetryA.update();

        follower.telemetryDebug(telemetryA);
    }
}
