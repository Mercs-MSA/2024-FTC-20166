package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPoseRight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;


@TeleOp
//@Disabled
public class DriveTestFaceGoal extends LinearOpMode {

    private double driveX;
    private double driveY;
    SparkFunOTOS myOtos;

    //Motor demo variables
    private DcMotorEx m0 = null;
    private DcMotorEx m1 = null;
    private DcMotorEx m2 = null;
    private DcMotorEx m3 = null;
//    private DcMotorEx m4 = null;
//    private DcMotorEx m5 = null;
//    private DcMotorEx m6 = null;
//    private DcMotorEx m7 = null;
    private IMU imu;

    private double FLYPower = 0.0;
    private double FRYPower = 0.0;
    private double BLYPower = 0.0;
    private double BRYPower = 0.0;
    private double FLXPower = 0.0;
    private double FRXPower = 0.0;
    private double BLXPower = 0.0;
    private double BRXPower = 0.0;

    private double FLRPower = 0.0;
    private double FRRPower = 0.0;
    private double BLRPower = 0.0;
    private double BRRPower = 0.0;

    private double driveRotate;

    public static SparkFunOTOS.Pose2D startPointMiddleBottom = new SparkFunOTOS.Pose2D(0, 60, Math.toRadians(0));
    private boolean currentlyTurning = false;
    private double desiredHeading;

    //public Point startPose

    public void initializeHardware()
    {
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        myOtos.setPosition(startPointMiddleBottom);
       // myOtos.
        m0 = hardwareMap.get(DcMotorEx.class, "FL");
        m1 = hardwareMap.get(DcMotorEx.class, "FR");
        m2 = hardwareMap.get(DcMotorEx.class, "BL");
        m3 = hardwareMap.get(DcMotorEx.class, "BR");
//        m4 = hardwareMap.get(DcMotorEx.class, "M4");
//        m5 = hardwareMap.get(DcMotorEx.class, "M5");
//        m6 = hardwareMap.get(DcMotorEx.class, "M6");
//        m7 = hardwareMap.get(DcMotorEx.class, "M7");

        m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        m7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.resetYaw();
    }

    public double getHeadingDegrees() {
        //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // return orientation.getYaw(AngleUnit.DEGREES);
        SparkFunOTOS.Pose2D pos = myOtos.getPosition();
        return pos.h;
    }
    private void setDriveMotors(double FL, double FR, double BL, double BR)
    {
        double greatest = Math.max(Math.max(FL, FR), Math.max(BL, BR));
        if (greatest > 1.0)
        {
            FL = FL/greatest;
            FR = FR/greatest;
            BL = BL/greatest;
            BR = BR/greatest;
        }
        m0.setPower(FL);
        m1.setPower(FR);
        m2.setPower(BL);
        m3.setPower(BR);

    }
    
    private void updateDriveControls() 
    {
        double angleInRadians;
        double oldDriveX = gamepad1.left_stick_x;
        double oldDriveY = gamepad1.left_stick_y;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        if(gamepad1.right_bumper)
        {
            angleInRadians = 0;
        }
        else
        {
            angleInRadians = orientation.getYaw(AngleUnit.RADIANS);
        }

        driveX = oldDriveX * Math.cos(angleInRadians) - oldDriveY * Math.sin(angleInRadians);
        driveY = oldDriveX * Math.sin(angleInRadians) + oldDriveY * Math.cos(angleInRadians);
        driveRotate = gamepad1.right_stick_x;
        if (Math.abs(driveRotate) < .1)
        {
            driveRotate = 0;
            if (currentlyTurning)
            {
                desiredHeading = getHeadingDegrees();
            }
            currentlyTurning = false;
        }
        else
        {
            currentlyTurning = true;
        }
        //double error = headingError(getHeadingDegrees(), desiredHeading);
//        if (error > 5)
//        {
//            //driveRotate =
//        }

    }

    private void calculateDrivePower()
    {
        FLYPower = -driveY;
        FRYPower = -driveY;
        BLYPower = -driveY;
        BRYPower = -driveY;

        FLXPower = driveX;
        FRXPower = -driveX;
        BLXPower = -driveX;
        BRXPower = driveX;


        FLRPower = driveRotate; //gamepad1.right_stick_x;
        FRRPower = -driveRotate; //-gamepad1.right_stick_x;
        BLRPower = driveRotate; //gamepad1.right_stick_x;
        BRRPower = -driveRotate; //-gamepad1.right_stick_x;
    }

    public static double getPointsHeading(double x, double y, double xr, double yr)
    {
        double calculatedAngleRads = Math.atan2(y - yr, x - xr);
        double calculatedAngleDegs = (180.0 / Math.PI) * calculatedAngleRads;
        double correctedAngle = calculatedAngleDegs - 90.0;
        return correctedAngle;
    }

    public void runOpMode() throws InterruptedException {
       initializeHardware();

        waitForStart();
        while (opModeIsActive())
        {
 /*           telemetry.addData("Motor 0", m0.getCurrentPosition());
            telemetry.addData("Motor 1", m1.getCurrentPosition());
            telemetry.addData("Motor 2", m2.getCurrentPosition());
            telemetry.addData("Motor 3", m3.getCurrentPosition());
            telemetry.addData("Motor 4", m4.getCurrentPosition());
            telemetry.addData("Motor 5", m5.getCurrentPosition());
            telemetry.addData("Motor 6", m6.getCurrentPosition());
            telemetry.addData("Motor 7", m7.getCurrentPosition());

  */
//            telemetry.addData("IMU X", orientation.getYaw(AngleUnit.DEGREES));
//            telemetry.addData("IMU Y", orientation.getPitch(AngleUnit.DEGREES));
//            telemetry.addData("IMU Z", orientation.getRoll(AngleUnit.DEGREES));
            updateDriveControls();
            calculateDrivePower();

            //FLXPower.setVelocity(1000);
            //Sets target velocity to 1000 ticks per second
            //m0.setVelocity();
            setDriveMotors((FLXPower + FLYPower + FLRPower), (FRXPower + FRYPower + FRRPower), (BLXPower + BLYPower + BLRPower), (BRXPower + BRYPower + BRRPower));

            SparkFunOTOS.Pose2D pos = myOtos.getPosition();
            telemetry.addData("X coordinate", pos.x);
            telemetry.addData("Y coordinate", pos.y);
            telemetry.addData("Heading angle", pos.h);

            updateTelemetry(telemetry);
        }
    }
}




