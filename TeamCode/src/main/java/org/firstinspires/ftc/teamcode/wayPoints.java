package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.startingPose;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class wayPoints {
    public static final Point startPoint = new Point (startingPose.getX(), startingPose.getY(), Point.CARTESIAN);
    public static final Point submersibleDropPoint = new Point(4.5, -33, Point.CARTESIAN); //Heading 1.542 (90)
    public static final double submersibleDropPointHeading = Math.toRadians(90);
    public static final Point spikeOneMidPoint = new Point(13, -49.4, Point.CARTESIAN); //Heading 0.811 (45)
    public static final double spikeOneMidPointHeading = Math.toRadians(45);
    public static final Point spikeOne = new Point(53, -37.2, Point.CARTESIAN); //Heading 1.493 (90)
    public static final double spikeOneHeading = Math.toRadians(90);
    public static final Point spikeOneBackOffOne = new Point(52.3, -44.7, Point.CARTESIAN); //Heading 1.493 (90)
    public static final double spikeOneBackOffOneHeading = 90;
    public static final Point dropOff = new Point(57.4, -57.6, Point.CARTESIAN); //Heading 4.659 (266)
    public static final double dropOffHeading = Math.toRadians(266);
    public static final Point pickupSpecimenMid = new Point(34.9, -58.3, Point.CARTESIAN); //Heading 4.669 (266)
    public static final Point pickUpSpecimen = new Point(34.6, -60, Point.CARTESIAN); //Heading 4.648 (268)
    public static double pickUpSpecimenHeading = Math.toRadians(266);
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
}
