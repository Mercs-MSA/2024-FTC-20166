package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    private RobotConstants constants = new RobotConstants();
    private Telemetry telemetryA;
    private SubSystemGrabber robotGrabber;
    private SubSystemElevator robotElevator;

    public double botHeading = startingPose.getHeading();
    private Follower follower;

    private enum AUTON_STATE
    {
        AUTON_START_STATE,
        LOWER_ELEVATOR_STATE,
        LOWER_ELEVATOR_SETUP_STATE,
        WAIT_AUTO_FINISHED,
        WAIT_PATH_DONE_STATE,
        PUSH_SAMPLES_STATE,
    }

    private AUTON_STATE currentAutonomousState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE waitPathDoneNextState = AUTON_STATE.AUTON_START_STATE;
    private AUTON_STATE lowerElevatorNextState = AUTON_STATE.AUTON_START_STATE;

    private static ElapsedTime timeoutTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int timeoutPeriod = 0;

    public static final Point startPoint = new Point (startingPose.getX(), startingPose.getY(), Point.CARTESIAN);
    public static final Point submersibleDropPoint = new Point(2.4, -32.6, Point.CARTESIAN);

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
    //Paths
    public static final Path submersibleToSpikePathSegment1 = new Path(new BezierCurve(submersibleDropPoint, submersibleToSpike1, submersibleToSpike2));
    public static final Path submersibleToSpikePathSegment2 = new Path(new BezierCurve(submersibleToSpike2, submersibleToSpike3));
    public static final Path submersibleToSpikePathSegment3 = new Path(new BezierCurve(submersibleToSpike3, submersibleToSpike4));
    public static final Path submersibleToSpikePathSegment4 = new Path(new BezierCurve(submersibleToSpike4, submersibleToSpike5, submersibleToSpike6));
    public static final Path submersibleToSpikePathSegment5 = new Path(new BezierCurve(submersibleToSpike6, submersibleToSpike7));
    public static final Path submersibleToSpikePathSegment6 = new Path(new BezierCurve(submersibleToSpike7, submersibleToSpike8));
    public static final Path submersibleToSpikePathSegment7 = new Path(new BezierCurve(submersibleToSpike8, submersibleToSpike9));
    public static final PathChain submersibleToSpikeOneChain = new PathChain(submersibleToSpikePathSegment1, submersibleToSpikePathSegment2, submersibleToSpikePathSegment3, submersibleToSpikePathSegment4, submersibleToSpikePathSegment5, submersibleToSpikePathSegment6, submersibleToSpikePathSegment7);
    public static  final Path startToSubmersible = new Path(new BezierCurve(startPoint, submersibleDropPoint));
    public static final double startToSubmersibleHeading = Math.toRadians(90);
    public static final double submersibleToSpike1Heading = Math.toRadians(90);


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init()
    {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.setMaxPower(constants.START_TO_SUBMERSIBLE_SPEED);
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
        follower.followPath(pathToFollow);
        pathToFollow.setLinearHeadingInterpolation(currentHeading, endHeading);
        botHeading = endHeading;
    }
    private void setupPathChain(PathChain pathChainToFollow, double endHeading)
    {
        double currentHeading = botHeading;
        follower.followPath(pathChainToFollow);
        botHeading = endHeading;
        int curveCount = pathChainToFollow.size();
        for (int i = 0; i < curveCount; i++)
        {
            pathChainToFollow.getPath(i).setLinearHeadingInterpolation(currentHeading, endHeading);
        }
    }

    private void restartTimeout(int timeout)
    {
        timeoutPeriod = timeout;
        timeoutTimer.reset();
    }
    private boolean pathIsBusy()
    {
        if (hasTimededout())
            return false;
        else if (follower.isBusy())
            return true;
        else
            return false;
    }

    private boolean hasTimededout()
    {
        if (timeoutTimer.time() < timeoutPeriod)
            return false;
        else
            return true;
    }

    private void processStateStart()
    {
        setupPath(startToSubmersible, startToSubmersibleHeading);
        restartTimeout(1000);
        robotElevator.setPosition(constants.ELEVATOR_TOP_RUNG_PLACE);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.LOWER_ELEVATOR_SETUP_STATE;
        lowerElevatorNextState = AUTON_STATE.PUSH_SAMPLES_STATE;
    }

    private void processLowerElevatorSetupState()
    {
        robotElevator.setPosition(constants.ELEVATOR_TOP_RUNG_RELEASE);
        restartTimeout(600);
        currentAutonomousState =AUTON_STATE.LOWER_ELEVATOR_STATE;
    }
    private void processLowerElevatorState()
    {
        if((Math.abs(robotElevator.getPosition() - constants.ELEVATOR_TOP_RUNG_RELEASE) < 20) || hasTimededout())
        {
            robotGrabber.setPosition(constants.GRABBER_OPEN_POSITION);
            currentAutonomousState = lowerElevatorNextState;
            robotElevator.setPosition(constants.ELEVATOR_BOTTOM_POSITION);
            follower.setMaxPower(constants.SUBMERSIBLE_TO_PUSH_SPEED);
        }
    }

    private void processWaitPathDone()
    {
        if (!pathIsBusy())
            currentAutonomousState = waitPathDoneNextState;
    }
    private void processWaitDoneState()
    {

    }

    private void processPushSampleToObservation()
    {
        setupPathChain(submersibleToSpikeOneChain, submersibleToSpike1Heading);
        currentAutonomousState = AUTON_STATE.WAIT_PATH_DONE_STATE;
        waitPathDoneNextState = AUTON_STATE.WAIT_AUTO_FINISHED;
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
            case WAIT_AUTO_FINISHED:
                processWaitDoneState();
                break;
            case LOWER_ELEVATOR_SETUP_STATE:
                processLowerElevatorSetupState();
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

        follower.telemetryDebug(telemetryA);
    }
}
