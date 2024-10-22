package org.firstinspires.ftc.teamcode;
// Fl                        control            Motor 0                 frontLeft Drive motor
// FR                        control            Motor 1                 frontRight Drive motor
// BL                        control            Motor 2                 backLeft Drive motor
// BR                        control            Motor 3                 backRight Drive motor
// grabberLeft               control            servo 0                 grabberLeft servo
// grabberRight              control            servo 1                 grabberRight servo
// blinkIn                   control            servo 2                 ledDriver
// driveServo                control            servo 3                 driveIntakeServo servo
// leftIntakeServo           control            servo 4                 leftIntakeArmServo servo
// rightIntakeServo          control            servo 5                 rightIntakeArmServo servo
// elevator                  expansion          motor 0                 elevator motor
// imu                       control            i2cBus 0                revInternalIMU
// leftDistanceSensor        control            i2Bus 1                 leftDistance sensor
// colorSensor               control            i2Bus 2                 color sensor
// frontDistanceSensor       control            i2cBus 3                frontDistance sensor



import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemElevator;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemGrabber;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntake;
import org.firstinspires.ftc.teamcode.Subsystems.SubSystemIntakeArm;


@TeleOp
public class DEEP_TeleOp_Main_20166 extends LinearOpMode {

    private RobotConstants constants = new RobotConstants();
    private int elevatorMoveTo = 0;
    private double translateX;
    private double translateY;
    private double joy1RightX;
    private double joy1RightY;//
    private double FLMP, FRMP, BLMP, BRMP;

    private boolean elevatorMoveBottom = false;
    private boolean elevatorMoveTop = false;
    private boolean elevatorMoveLow = false;
    private boolean elevatorMoveHigh = false;

    private boolean grabberOpen;
    private boolean grabberClose;

    private double intakeSpeed = 0;
private double intakeArmPosition = 0;
    //Motor demo variables
    private DcMotorEx frontLeftDrive = null;
    private DcMotorEx frontRightDrive = null;
    private DcMotorEx backLeftDrive = null;
    private DcMotorEx backRightDrive = null;
    private IMU imu         = null;

    private SubSystemElevator robotElevator = null;
    private SubSystemGrabber robotGrabber = null;
    private SubSystemIntakeArm robotIntakeArm = null;
    private SubSystemIntake robotIntake = null;
    NormalizedColorSensor colorSensor;
    RevBlinkinLedDriver blinkinLedDriver;
    Rev2mDistanceSensor leftDistanceSensor;
    Rev2mDistanceSensor frontDistanceSensor;

    private boolean driverAssistPickup = false;

    public void initializeDriveMotors()
    {
        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "FL");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "FR");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "BL");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "BR");

        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void initializeSensors()
    {
        //Initialize gyro etc...
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        //Initialize the color sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        //Initialize distance sensors
        leftDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "leftDistanceSensor");
        frontDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor");

    }

    private void initializeSubSystems() throws InterruptedException {
        robotElevator = new SubSystemElevator(hardwareMap, constants.ELEVATOR_MULTIPLIER);
        robotGrabber = new SubSystemGrabber(hardwareMap);
        robotGrabber.setPosition(constants.GRABBER_OPEN_POSITION);
        robotIntake = new SubSystemIntake(hardwareMap);
        robotIntakeArm = new SubSystemIntakeArm(hardwareMap);
    }

    private void initializeLEDs(){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }

    private void initalizeEverything() throws InterruptedException {
        initializeDriveMotors();
        initializeSensors();
        initializeSubSystems();
        initializeLEDs();
    }

    public String getSampleColor()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
        float maxsat = Math.max(Math.max(colors.red, colors.green), colors.blue);
        float r = colors.red/maxsat;
        float g = colors.green/maxsat;
        float b = colors.blue/maxsat;

        String sample = "Nothing";

        if (distance < 3.0) {
            if (r == 1.0) {
                sample = "Red";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (b == 1.00) {
                sample = "Blue";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            } else {
                sample = "Yellow";
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
        }
        else {
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        }
        return sample;
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private static double[] rotatePoint(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        Result[0] = xPoint * Math.cos(angle) - yPoint * Math.sin(angle);
        Result[1] = xPoint * Math.sin(angle) + yPoint * Math.cos(angle);
        return Result;
    }

    private static double rotatePointX(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        return xPoint * Math.cos(angle) - yPoint * Math.sin(angle);
    }

    private static double rotatePointY(double xPoint, double yPoint, double angle) {
        double[]  Result = new double[2];

        return xPoint * Math.sin(angle) + yPoint * Math.cos(angle);
    }

    private void updateJoysticks()
    {
        //GAMEPAD1:
        //dpad up - Moving intake arm up
        //dpad down - Moving intake arm down
        //dpad left -
        //dpad right -
        //a -
        //b -
        //y -
        //x - Driver Assist
        //left bumper (lb) -
        //right bumper (rb) - Switch to robot centric
        //left trigger (lt) -
        //right trigger (rt) -
        //left joystick - Translate robot
        //right joystick x - Rotating robot
        //right joystick y -
        //GAMEPAD2:
        //dpad up - elevatorMoveTop
        //dpad down - elevatorMoveBottom
        //dpad left - elevatorMoveLow
        //dpad right - elevatorMoveHigh
        //a -
        //b - grabberClose
        //y -
        //x - grabberOpen
        //left bumper (lb) - INTAKE (NEED TO SEE WHICH WAY IT TURNS)
        //right bumper (rb) - INTAKE (NEED TO SEE WHICH WAY IT TURNS)
        //left trigger (lt) -
        //right trigger (rt) -
        //left joystick -
        //right joystick -
        //driving
        double x, y, heading;
        heading = getHeading();

        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        joy1RightX = gamepad1.right_stick_x;
        joy1RightY = gamepad1.right_stick_y;

        if (gamepad1.right_bumper){//Switch to robot centric
            translateX = x;
            translateY = y;
        }
        else{
            translateX = rotatePointX(x, y, heading);
            translateY = rotatePointY(x, y, heading);
        }

        elevatorMoveBottom = gamepad2.dpad_down;
        elevatorMoveTop = gamepad2.dpad_up;
        elevatorMoveLow = gamepad2.dpad_left;
        elevatorMoveHigh = gamepad2.dpad_right;

        grabberOpen = gamepad2.x;
        grabberClose = gamepad2.b;

        if (gamepad1.x)
            driverAssistPickup = true;
        else
            driverAssistPickup = false;

        if (gamepad2.right_bumper)
            intakeSpeed = 1;
        else if (gamepad2.left_bumper)
            intakeSpeed = -1;
        else
            intakeSpeed = 0;
        //buttons

        if (gamepad1.dpad_up)
            intakeArmPosition = constants.INTAKE_ARM_UP_POSITION;
        else if (gamepad1.dpad_down)
            intakeArmPosition = constants.INTAKE_ARM_DOWN_POSITION;

        //if(gamepad1.left_bumper) arm.setPosition(-786); else arm.setPosition(0);
    }

    private void updateDashboard()
    {
//        telemetry.addData("FL",frontLeftDrive.getCurrentPosition());
//        telemetry.addData("FR",frontRightDrive.getCurrentPosition());
//        telemetry.addData("BL",backLeftDrive.getCurrentPosition());
//        telemetry.addData("BR",backRightDrive.getCurrentPosition());
        telemetry.addData("Heading", getHeadingDegrees());

        telemetry.addLine("\n");
        telemetry.addData("Elevator Pos", robotElevator.getPosition());
        telemetry.addData("Elevator moveto", elevatorMoveTo);

        telemetry.addLine("\n");
        telemetry.addData("Sample detected", getSampleColor());

        telemetry.addData("Left distance", leftDistanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Front distance", frontDistanceSensor.getDistance(DistanceUnit.MM));
        updateTelemetry(telemetry);
    }
    private void updateDrivebaseMotors(double FLMP, double FRMP, double BLMP, double BRMP)
    {
        double maxPower;

        maxPower = Math.max(Math.max(FLMP, FRMP), Math.max(BLMP, BRMP));

        if (maxPower > 1.0)
        {
            FLMP = FLMP / maxPower;
            FRMP = FRMP / maxPower;
            BLMP = BLMP / maxPower;
            BRMP = BRMP / maxPower;
        }
        frontLeftDrive.setPower(FLMP);
        frontRightDrive.setPower(FRMP);
        backLeftDrive.setPower(BLMP);
        backRightDrive.setPower(BRMP);
    }
    private void calculateDrivebaseSpeed()
    {
        double FLPFB, FRPFB, BLPFB, BRPFB;
        double FLPLR, FRPLR, BLPLR, BRPLR;
        double FLPR, FRPR, BLPR, BRPR;

        FLPFB = translateY;
        FRPFB = translateY;
        BLPFB = translateY;
        BRPFB = translateY;

        FLPLR = -translateX;
        FRPLR = translateX;
        BLPLR = translateX;
        BRPLR = -translateX;

        FLPR = -joy1RightX;
        FRPR = joy1RightX;
        BLPR = -joy1RightX;
        BRPR = joy1RightX;

        FLMP = FLPFB + FLPLR + FLPR;
        FRMP = FRPFB + FRPLR + FRPR;
        BLMP = BLPFB + BLPLR + BLPR;
        BRMP = BRPFB + BRPLR + BRPR;
    }
    private void setElevator(int position)
    {
        robotElevator.setPosition(position);
        elevatorMoveTo = position;
    }
    public void updateElevator()
    {
        if (elevatorMoveBottom == true)
        {
            setElevator(constants.ELEVATOR_BOTTOM_POSITION);
        }
        else if (elevatorMoveTop == true)
        {
            setElevator(constants.ELEVATOR_TOP_RUNG_PLACE);
        }
        else if (elevatorMoveLow == true)
        {
            setElevator(constants.ELEVATOR_SPECIMEN_PICKUP);
        }
        else if (elevatorMoveHigh == true)
        {
            setElevator(constants.ELEVATOR_TOP_RUNG_RELEASE);
        }
        else if (gamepad2.right_bumper == true)
        {
            setElevator(robotElevator.getPosition() + constants.ELEVATOR_TEST_CHANGE);
        }
        else if (gamepad2.left_bumper == true)
        {
            setElevator(robotElevator.getPosition() - constants.ELEVATOR_TEST_CHANGE);
        }
    }
    private void setGrabberServo (boolean state)
    {
        if (state)
            robotGrabber.setPosition(constants.GRABBER_OPEN_POSITION);
        else
            robotGrabber.setPosition(constants.GRABBER_CLOSE_POSITION);
    }
    public void updateGrabber()
    {
       if (grabberOpen == true)
       {
           setGrabberServo(true);
       }
       else if (grabberClose == true)
        {
            setGrabberServo(false);
        }
    }

    private boolean rotateRobotToHeading(double heading)
    {
        double rotateSpeed = 0.45;
        double tolerance = 3;
        double headingError = getHeadingDegrees() - heading;
        if (headingError < -180)
            headingError = headingError + 360;
        else if (headingError > 180)
            headingError = headingError - 360;
        if (headingError < -tolerance)
        {
            updateDrivebaseMotors(rotateSpeed, -rotateSpeed, rotateSpeed,-rotateSpeed);
            return false;
        }
        else if (headingError > tolerance)
        {
            updateDrivebaseMotors(-rotateSpeed, rotateSpeed, -rotateSpeed, rotateSpeed);
            return false;
        }
        else
        {
            updateDrivebaseMotors(0, 0, 0, 0);
            return true;
        }
    }
    private boolean driveRobotToDistanceFrom (double distance)
    {
        double currentDistance = frontDistanceSensor.getDistance(DistanceUnit.MM);
        double driveSpeed = -0.25;
        if (currentDistance > distance)
        {
            updateDrivebaseMotors(driveSpeed, driveSpeed, driveSpeed, driveSpeed);
            return false;

        }
            else
        {
            updateDrivebaseMotors(0, 0, 0, 0);
            return true;
        }
    }
    private DRIVER_ASSIST_STATE currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT;
    private DRIVER_ASSIST_STATE returnDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT;
    private enum DRIVER_ASSIST_STATE {STATE_WAIT, STATE_FACE_WALL, STATE_DRIVE_TO_WALL_1, STATE_WAIT_RELEASE, STATE_STRAFE_TO_WALL, STATE_CLOSE_AND_TURN};
    private void processStateWait()
    {
        if (driverAssistPickup)
        {
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_FACE_WALL;
        }
    }
    private void processStateFaceWall()
    {
        boolean done = rotateRobotToHeading(180.0);
        if (done)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_DRIVE_TO_WALL_1;

    }
    private void processStateDriveToWall()
    {
        boolean done = driveRobotToDistanceFrom(120.0);
        if (done)
        {
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_STRAFE_TO_WALL;
            setGrabberServo(true);
            setElevator(constants.ELEVATOR_SPECIMEN_PICKUP);
        }
    }
    private void processStateWaitRelease()
    {
        if (!driverAssistPickup)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT;
    }
    private void processStateToWall()
    {
        boolean done = true;
        if (done)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT_RELEASE;
    }
    private void processStateCloseAndTurn()
    {
        boolean done = true;
        if (done)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT_RELEASE;
    }
    private void processStateStrafeToWall()
    {
        boolean done = true;
        if (done)
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT_RELEASE;
    }
    private void processStateDelay()
    {
        boolean done = true;
        if (done)
            currentDriverAssistState = returnDriverAssistState;
    }





    private void processStateMachine()
    {
        boolean done = false;
        if (!driverAssistPickup)
        {
            currentDriverAssistState = DRIVER_ASSIST_STATE.STATE_WAIT;

        }
        else
        {
            switch (currentDriverAssistState) {
                case STATE_WAIT:
                    processStateWait();
                    break;
                case STATE_FACE_WALL:
                    processStateFaceWall();
                    break;
                case STATE_DRIVE_TO_WALL_1:
                    processStateDriveToWall();
                    break;
                case STATE_STRAFE_TO_WALL:
                    processStateStrafeToWall();
                    break;
                case STATE_CLOSE_AND_TURN:
                    processStateCloseAndTurn();
                    break;
                case STATE_WAIT_RELEASE:
                    processStateWaitRelease();
                    break;
            }

        }


    }

    private void updateIntake()
    {
        robotIntake.setSpeed(intakeSpeed);
        telemetry.addData("intakeServo", intakeSpeed);

    }
    private void updateIntakeArm()
    {
        robotIntakeArm.setPosition(intakeArmPosition);

    }

    public void runOpMode() throws InterruptedException {
        initalizeEverything();

        waitForStart();

        while (opModeIsActive())
        {
            updateJoysticks();
            calculateDrivebaseSpeed();
            if (currentDriverAssistState == DRIVER_ASSIST_STATE.STATE_WAIT)
            {
                updateDrivebaseMotors(FLMP, FRMP, BLMP, BRMP);
                updateElevator();
                updateGrabber();
                updateIntake();
                updateIntakeArm();
            }
            processStateMachine();
            updateDashboard();
        }

    }
}




