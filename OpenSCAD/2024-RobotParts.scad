//Make sure the system variable OPENSCADPATH  points to the Robotics\Library location

//use     <D:\3D Printer & Laser Cutter\3D models\Robotics\Library\Getriebe.scad>
//use <D:\3D Printer & Laser Cutter\3D models\Robotics\Library\RobotPrimitives.scad>
//use     <D:\3D Printer & Laser Cutter\3D models\Robotics\Library\Sprockets.scad>

//Included so global parameters are defined
//include <RobotPrimitives.scad>
//use <Getriebe.scad>
//use <Sprockets.scad>
//use <threads.scad>

// sprocket(size, teeth, bore, hub_diameter, hub_height, guideangle);
//Include the hopper?
$DoHopper = true;
//Show the servos?
$DoServo = true;
//Diameter of the gripper arm mount cylinders used to attach the servo horns
$SampleServoMountDiameter = 32;
//Base sample gripper frame width
$SampleGrabberFrameWidth = 180;
//Additional spacing between the 'pyramids' to allow the sample to fit
$SampleGrabberGapAdder = 6;
//Size of the 'pyramid'
$SampleGrabberInsertWidth = 34;
//Grabber rotation clearance distance from frame to allow rotation and hopper
$SampleGrabberPushoutDistance = 10;
//Grabber center to center
$SampleGrabberSpacing = $SampleGrabberGapAdder + $SampleGrabberInsertWidth + $SampleServoMountDiameter + $SampleGrabberPushoutDistance + $SampleGrabberPushoutDistance;
//Thickness of the grabber frame
$SampleGrabberFrameThickness = 5;
//Distance from tip of servo shaft to top of cylinder
$SampleGrabberHornClearance = 5;
$SampleGrabberHornClearanceAdjusted = .8 + $SampleGrabberHornClearance;// (0.8 adjustment for specific servos)
//Pyramid drop distance to get closer to the floor
$SampleGrabberDropDistance = 20;
//Length of the grabber arms
$SampleGrabberArmLength = 60;
//Sample grabber open/close rotation
$SampleGrabberArmRotation = -20;

module _block_customizer(){}

$fn = 100;

$Hex2Circle = 1/(sin(60));
$Inch2mm = 25.5;
$ServoRotateOffset = 9.85;

$M4ThreadlockNutDiameterFlat = 6.4;
$M4ThreadlockNutDiameterRound = 7.9;
$M4ThreadlockNutHeight = 4.8;
$M4NonThreadedD = 4.3;
$M4ThreadedD = 4.00 - 0.05;

$M3NonThreadedD = 3.2;
$M3ThreadedD = 4.00 - 0.05;

module Servo()
{
  translate([$ServoRotateOffset, 0, 0])
    import("GoBildaServoLoRes.stl");
}

module SpindleCore(InnerD, OuterD, Height, RimHeight, SlopeSpan, ShaftD, ShaftFaces, ThreadD)
{
  difference()
  {
    union()
    {
      //Central core
      cylinder(d = InnerD, h = Height);
      //Bottom rim
      cylinder(d = OuterD, h = RimHeight);
      translate([0, 0, RimHeight])
        cylinder(d1 = OuterD, d2 = InnerD, h = SlopeSpan);

      //Top rim
      translate([0, 0, Height - RimHeight])
        cylinder(d = OuterD, h = RimHeight);
      translate([0, 0, Height - RimHeight - SlopeSpan])
        cylinder(d2 = OuterD, d1 = InnerD, h = SlopeSpan);
    }
    cylinder(d = ShaftD, h = Height, $fn = ShaftFaces);
    translate([InnerD / 4, 0, Height / 2])
      rotate(90, [1, 0, 0])
        cylinder(d = ThreadD, h = OuterD, center = true);
  }
}

module FTCLifterSpindle(Splitter = false)
{  
  $HubDiameter = $SpindleDiameter + 5;
  $HubDepth = 8;
  $FlatOffset = 0.5;
  $NutWidth = 5.5 + 0.5;
  $NutThickness = 1.9 + 0.5;
  $NutBore = 3.2 + 0.2;
  $NutOffset = 6;
 
  difference()
  { 
    union()
    {
      //Hub
      cylinder(d = $HubDiameter, h = $HubDepth - 2, $fn = 50);
      translate([0, 0, $HubDepth - 2])
        cylinder(d1 = $HubDiameter, d2 = $SpindleDiameter, h = 2, $fn = 50);
      //Thread guide
      translate([0, 0, $HubDepth])
        //#1 = 3.3, #2 = 4
        if ($SpindleType == 0)
          metric_thread (diameter=$SpindleDiameter, pitch=4, length=$SpindleLength, thread_size=3, groove=false);
        else
          cylinder(d = $SpindleDiameter, h = $SpindleLength);
      difference()
      {
        //Hub
        translate([0, 0, $HubDepth + $SpindleLength])
        {
          translate([0, 0, 2])
            cylinder(d = $HubDiameter, h = $HubDepth - 2, $fn = 50);
          cylinder(d2 = $HubDiameter, d1 = $SpindleDiameter, h = 2, $fn = 50);
        }
        //Cable tieoff
        translate([0, -5, $HubDepth + $HubDepth + $SpindleLength + 3])
          rotate(145, [1, 0, 0])
            cylinder(d = 4, h = 16, $fn = 50);
      }
     if (Splitter)
      {
        $SplitterD = 2;
        translate([0, 0, ($SpindleLength / 2) + $HubDepth - $SplitterD])
          cylinder(d2 = $HubDiameter, d1 = $SpindleDiameter, h = $SplitterD, $fn = 50);
        translate([0, 0, ($SpindleLength / 2) + $HubDepth])
          cylinder(d1 = $HubDiameter, d2 = $SpindleDiameter, h = $SplitterD, $fn = 50);
      }
    }
     if (Splitter)
      {
      }
    //Motor shaft opening
    if ($ShaftType == 0)
      difference()
      {
        //Shaft bore
        cylinder(d = $ShaftDiameter, h = $HubDepth + $HubDepth + $SpindleLength, $fn = 50);
        //Shaft bode flat side
        translate([-$ShaftDiameter/2, ($ShaftDiameter / 2) - $FlatOffset, 0])
          cube([$ShaftDiameter, $ShaftDiameter, $HubDepth + $HubDepth + $SpindleLength]);
      }
    else
      //Shaft bore
      cylinder(d = $ShaftDiameter, h = $HubDepth + $HubDepth + $SpindleLength, $fn = 6);
      
    //Lock bolt shaft openings
    translate([0, 0, $HubDepth / 2])
    {
      rotate(-90, [1, 0, 0])
        cylinder(d = $NutBore, h = $SpindleDiameter, $fn = 50);
      translate([0, 0, $HubDepth + $SpindleLength])
        rotate(-90, [1, 0, 0])
          cylinder(d = $NutBore, h = $SpindleDiameter, $fn = 50);
    }
    //Lock bolt nut openings
    translate([0, $NutOffset, $HubDepth / 2])
      cube([$NutWidth, $NutThickness, $HubDepth], center = true);
    translate([0, $NutOffset, ($HubDepth / 2) + $SpindleLength + $HubDepth])
      cube([$NutWidth, $NutThickness, $HubDepth], center = true);
  }
}

module PulleyWheel()
{
  translate([-2, 0, 0])
    rotate(90, [0, 1, 0])
      difference()
      {
        union()
        {
          cylinder(d = 12, h = 1);
          translate([0, 0, 1])
            cylinder(d1 = 12, d2 = 9.6, h = 1);
          translate([0, 0, 2])
            cylinder(d2 = 12, d1 = 9.6, h = 1);
          translate([0, 0, 3])
            cylinder(d = 12, h = 1);
        }
        translate([0, 0, -0.5])
          cylinder(d = 3.2, h = 5);
      }
}

 
module SampleGrabberArmPair()
{
  translate([0, ($SampleGrabberSpacing) / 2, 0])
    rotate($SampleGrabberArmRotation, [0, 0, 1])
      SampleGrabberArm();
  translate([0, -($SampleGrabberSpacing) / 2, 0])
    mirror([0, 1,0])
      rotate($SampleGrabberArmRotation, [0, 0, 1])
        SampleGrabberArm();
}

module SampleGrabberArm()
{
  $InsertDepth = 6;

  translate([-$SampleGrabberArmLength + ($SampleGrabberInsertWidth / 2), (-$SampleServoMountDiameter / 2) - $SampleGrabberPushoutDistance, 0])
    rotate(90, [1, 0, 0])
    {
      //Pyramid
      translate([0, -$SampleGrabberDropDistance, 0])
        rotate(45, [0, 0, 1])
          cylinder(d1 = $SampleGrabberInsertWidth * sqrt(2), d2 = 0, h = $SampleGrabberInsertWidth / 2, $fn = 4);
      //Pyramid drop support
      translate([0, -$SampleGrabberDropDistance / 2, (-$InsertDepth / 2)])
        cube([$SampleGrabberInsertWidth, $SampleGrabberInsertWidth + $SampleGrabberDropDistance, $InsertDepth], center = true);
      //Arm
      translate([(-$SampleGrabberInsertWidth / 2), -$SampleGrabberInsertWidth / 2, -$InsertDepth])
        cube([$SampleGrabberArmLength, $SampleGrabberInsertWidth, $InsertDepth]);
    }
  //Horn mount
  difference()
  {
    //Core
    hull()
    {
      cylinder(d = $SampleServoMountDiameter, h = $SampleGrabberInsertWidth, center = true);
      translate([0, (-$SampleServoMountDiameter / 2) - $SampleGrabberPushoutDistance + 0.5, 0])
        cube([20, 1, $SampleGrabberInsertWidth], center = true);
    }
    //Rotate a little so does not collide with arm block
    rotate(30, [0, 0, 1])
    {
      //Shaft hole
      cylinder(d = 4, h = $SampleGrabberInsertWidth + 1, center = true);
      //Horn mount holes
      translate([12, 0, 0])
        cylinder(d = 4, h = $SampleGrabberInsertWidth + 1, center = true);
      translate([-12, 0, 0])
        cylinder(d = 4, h = $SampleGrabberInsertWidth + 1, center = true);
      translate([0, 12, 0])
        cylinder(d = 4, h = $SampleGrabberInsertWidth + 1, center = true);
      translate([0, -12, 0])
        cylinder(d = 4, h = $SampleGrabberInsertWidth + 1, center = true);
    }
  }
  if ($DoHopper)
  {
    $HopperDepth = 50;
    translate([-$SampleGrabberArmLength + ($HopperDepth / 2), (-($SampleServoMountDiameter - $InsertDepth) / 2) - $SampleGrabberPushoutDistance, 50])
    {
      translate([0, 0, (40 / 2)])
      {
        translate([0, 0, -28])
          cube([$HopperDepth, $InsertDepth, 60], center = true);
        rotate(-40, [1, 0, 0])
          translate([0, 0, (40 / 2)])
            cube([$HopperDepth, $InsertDepth, 40], center = true);
      }
      translate([22, -20, 0])
        rotate(40, [0, 1, 0])
          cube([$InsertDepth, 30, 55]);
    }
  }
}

module ServoCutout($Height = 6)
{
  translate([0, $ServoRotateOffset, 0])
  {
    //Main body
    cube([21, 41, $Height], center = true);
    //Alignment grooves
    cube([2, 55, $Height], center = true);
    //Mount holes
    translate([5, 24, 0])
      cylinder(d = 4.5, h = $Height, center = true);
    translate([-5, 24, 0])
      cylinder(d = 4.5, h = $Height, center = true);
    translate([5, -24, 0])
      cylinder(d = 4.5, h = $Height, center = true);
    translate([-5, -24, 0])
      cylinder(d = 4.5, h = $Height, center = true);
  }
}

module SampleGrabberFrame()
{
  //Top servo mount
  translate([0, 0, (($SampleGrabberInsertWidth + $SampleGrabberFrameThickness)/ 2) + $SampleGrabberHornClearanceAdjusted + 8.5])
  {
    difference()
    {
      //Core
      cube([$SampleGrabberInsertWidth + 1, $SampleGrabberFrameWidth, $SampleGrabberFrameThickness], center = true);
      //Servo cutouts
      translate([0, ($SampleGrabberSpacing / 2), 0])
        ServoCutout();
      mirror([0, 1, 0])
        translate([0, ($SampleGrabberSpacing / 2), 0])
          ServoCutout();
    }
  }
  //Lower gripper support
  translate([0, 0, -($SampleGrabberInsertWidth + $SampleGrabberFrameThickness) / 2])
  {
    difference()
    {
      //Core
      cube([$SampleGrabberInsertWidth + 1, $SampleGrabberFrameWidth, $SampleGrabberFrameThickness], center = true);
      translate([0, ($SampleGrabberSpacing / 2), 0])
        cylinder(d = 3.5, h = $SampleGrabberFrameThickness + 1, center = true);
      translate([0, -($SampleGrabberSpacing / 2), 0])
        cylinder(d = 3.5, h = $SampleGrabberFrameThickness + 1, center = true);
    }
  }
  //Back support
  translate([($SampleGrabberFrameThickness + $SampleGrabberInsertWidth + 1) / 2, 0, -($SampleGrabberInsertWidth / 2) - $SampleGrabberFrameThickness])
  {
    difference()
    {
      //Core
      translate([-$SampleGrabberFrameThickness / 2, -$SampleGrabberFrameWidth / 2, 0])
        cube([$SampleGrabberFrameThickness, $SampleGrabberFrameWidth, $SampleGrabberFrameThickness + $SampleGrabberFrameThickness + $SampleGrabberInsertWidth + $SampleGrabberHornClearanceAdjusted + 8.5]);
      //Lifter attach bracket holes
      for (x = [-3:3])
        for (y = [0:2])
          translate([0, (x * 25), (y * 15) + 15])
            rotate(90, [0, 1, 0])
              cylinder(d = 2.9, h = $SampleGrabberFrameThickness + 1, center = true);
    }
  }
}

module SampleGrabberLifterAttach()
{
  difference()
  {
    cube([$SampleGrabberFrameThickness, 50, 50], center = true);
    //Lifter attach bracket holes
    for (x = [0:1])
      for (y = [0:2])
        translate([0, (x * 25) - 10, (y * 15) - 15])
          rotate(90, [0, 1, 0])
            cylinder(d = 2.9, h = $SampleGrabberFrameThickness + 1, center = true);
  }
  translate([(30 + $SampleGrabberFrameThickness)/ 2, -(50 - $SampleGrabberFrameThickness) / 2, 0])
    difference()
    {
      cube([30, $SampleGrabberFrameThickness, 50], center = true);
      translate([0, -$SampleGrabberFrameThickness + 0.6, 0])
        cube([20.1, $SampleGrabberFrameThickness, 51], center = true);
      rotate(90, [1, 0, 0])
        cylinder(d = 3.5, h = $SampleGrabberFrameThickness + 1, center = true);
    }
}

module SampleGrabberMechanism($Servo)
{
  SampleGrabberArmPair();
  SampleGrabberFrame();
  translate([(($SampleGrabberFrameThickness + $SampleGrabberInsertWidth + 1) / 2) + $SampleGrabberFrameThickness, -15, 8])
  SampleGrabberLifterAttach();
  
  if ($Servo)
  {
    color([0.4, 0.4, 0.4, 0.5])
    {
      translate([0, ($SampleGrabberSpacing) / 2, $SampleGrabberHornClearanceAdjusted + $SampleGrabberFrameThickness + ($SampleGrabberInsertWidth / 2)])
        rotate(90, [0, 0, 1])
          mirror([0, 0, 1])
            Servo();
      mirror([0, 1, 0])
      translate([0, ($SampleGrabberSpacing) / 2, $SampleGrabberHornClearanceAdjusted + $SampleGrabberFrameThickness + ($SampleGrabberInsertWidth / 2)])
          rotate(90, [0, 0, 1])
            mirror([0, 0, 1])
              Servo();
    }
  }
}

module BotBaseWheel()
{
  color("steelblue")
    rotate(90, [1, 0, 0])
      cylinder(d = 100, h = 20, center = true);
}

module BotBase()
{
  $BaseW = 425;
  $BaseL = 425;
  $WheelLOffset = 50;
  $WheelWOffset = 14;
  $BaseLSpacing = $BaseW - $WheelLOffset;
  $BaseWSpacing = $BaseL - $WheelWOffset;

  translate([$BaseLSpacing / 2, $BaseWSpacing / 2, 50])
    BotBaseWheel();
  translate([-$BaseLSpacing / 2, $BaseWSpacing / 2, 50])
    BotBaseWheel();
  translate([$BaseLSpacing / 2, -$BaseWSpacing / 2, 50])
    BotBaseWheel();
  translate([-$BaseLSpacing / 2, -$BaseWSpacing / 2, 50])
    BotBaseWheel();
  color("silver")
  translate([0, 0, 100 / 2])
  cube([$BaseW, $BaseL, 50], center = true);
}

  //Grab bar location
/*
  $ClimbArmLength = 12 * $Inch2mm;
  $ClimbArmVOffset = 10 * $Inch2mm;
  $ClimbArmHOffset = -3 * $Inch2mm;
  $ClimbArmAngle = 27;
  $BotHOffset = -9.7 * $Inch2mm;
  $BotVOffset = 0;
  $BotRotation = 0;
*/

  //Lift latch point
/*  
  $ClimbArmLength = 18 * $Inch2mm;
  $ClimbArmVOffset = 10 * $Inch2mm;
  $ClimbArmHOffset = -3 * $Inch2mm;
  $ClimbArmAngle = 27;
  $BotHOffset = 0 * $Inch2mm;
  $BotVOffset = 9 * $Inch2mm;
  $BotRotation = -30;
*/

module ClimbArm()
{
  difference()
  {
    translate([-20, -1.5, -30])
      cube([40, 3, $ClimbArmLength + 30 + 30]);
    rotate(90, [1, 0, 0])
      cylinder(d = 26, h = 5, center = true);
    translate([0, 0, $ClimbArmLength])
      rotate(90, [1, 0, 0])
        hull()
        {
          cylinder(d = 26, h = 5, center = true);
          translate([30, -10, 0])
            cylinder(d = 26, h = 5, center = true);
        }
  }
}

module ClimbTest()
{ 
  //Floor
  cube([2000, 2000, 0.01], center = true);
  
  //Climb bars
  color("blue")
  {
    translate([0, 0, 508])
      rotate(90, [1, 0, 0])
        cylinder(d = 26, h = 1000, center = true);
    translate([0, 0, 914])
      rotate(90, [1, 0, 0])
        cylinder(d = 26, h = 1000, center = true);
  }
  
  
  translate([$BotHOffset, 0, $BotVOffset])
  {
    rotate($BotRotation, [0, 1, 0])
    {
      translate([-$ClimbArmHOffset, 0, $ClimbArmVOffset])
        rotate($ClimbArmAngle, [0, 1, 0])
          ClimbArm();
      
//      translate([-$ClimbArmHOffset, 0, 0])
        BotBase();
      color([0.3, 0.1, 0.6, 0.1])
        translate([0, 0, 9 * $Inch2mm])
          cube([18 * $Inch2mm, 18 * $Inch2mm, 18 * $Inch2mm], center = true);
    }
  }
}

/*
  //Start climb position
  $ClimbSliderLength = 12 * $Inch2mm;
  $ClimbSliderLocation = [0, 0, 12 * $Inch2mm];
  $ClimbSliderRotation = -45;
  $BotHOffset = -8.5 * $Inch2mm;
  $BotVOffset = 0.0 * $Inch2mm;
  $BotRotation = 0;
*/

/*
  //Start of lift
  $ClimbSliderLength = 12 * $Inch2mm;
  $ClimbSliderLocation = [0, 0, 12 * $Inch2mm];
  $ClimbSliderRotation = -45;
  $BotHOffset = -9.5 * $Inch2mm;
  $BotVOffset = 1.0 * $Inch2mm;
  $BotRotation = 5;
*/

/*
  //Tip point
  $ClimbSliderLength = 12 * $Inch2mm;
  $ClimbSliderLocation = [0, 0, 12 * $Inch2mm];
  $ClimbSliderRotation = -45;
  $BotHOffset = -8.5 * $Inch2mm;
  $BotVOffset = 4.0 * $Inch2mm;
  $BotRotation = 10;
*/

/*
  //Post tip point
  $ClimbSliderLength = 12 * $Inch2mm;
  $ClimbSliderLocation = [0, 0, 12 * $Inch2mm];
  $ClimbSliderRotation = -45;
  $BotHOffset = -1.5 * $Inch2mm;
  $BotVOffset = 4.0 * $Inch2mm;
  $BotRotation = -10;
*/

  //Full pull up stage 1
  $ClimbSliderLength = 12 * $Inch2mm;
  $ClimbSliderLocation = [0, 0, 12 * $Inch2mm];
  $ClimbSliderRotation = -45;
  $BotHOffset = -1.5 * $Inch2mm;
  $BotVOffset = 8.0 * $Inch2mm;
  $BotRotation = 5;

module ClimbTest2()
{ 
  //Floor
  cube([2000, 2000, 0.01], center = true);
  
  //Climb bars
  color("blue")
  {
    translate([0, 0, 508])
      rotate(90, [1, 0, 0])
        cylinder(d = 26, h = 1000, center = true);
    translate([0, 0, 914])
      rotate(90, [1, 0, 0])
        cylinder(d = 26, h = 1000, center = true);
  }
  
  
  translate([$BotHOffset, 0, $BotVOffset])
  {
    rotate($BotRotation, [0, 1, 0])
    {
      translate($ClimbSliderLocation)
        rotate($ClimbSliderRotation, [0, 1, 0])
          cube([$ClimbSliderLength, 1, 25]);
//      translate([-$ClimbArmHOffset, 0, 0])
        BotBase();
      color([0.3, 0.1, 0.6, 0.1])
        translate([1 * 25.4, 0, 9 * $Inch2mm])
          cube([18 * $Inch2mm, 18 * $Inch2mm, 18 * $Inch2mm], center = true);
    }
  }
}

module BotBaseWheel()
{
  color("steelblue")
    rotate(90, [1, 0, 0])
      cylinder(d = 100, h = 20, center = true);
}

module BotBase()
{
  $BaseW = 14 * 25.4;
  $BaseL = 425;
  $WheelLOffset = 50;
  $WheelWOffset = 14;
  $BaseLSpacing = $BaseW - $WheelLOffset;
  $BaseWSpacing = $BaseL - $WheelWOffset;

  translate([$BaseLSpacing / 2, $BaseWSpacing / 2, 50])
    BotBaseWheel();
  translate([-$BaseLSpacing / 2, $BaseWSpacing / 2, 50])
    BotBaseWheel();
  translate([$BaseLSpacing / 2, -$BaseWSpacing / 2, 50])
    BotBaseWheel();
  translate([-$BaseLSpacing / 2, -$BaseWSpacing / 2, 50])
    BotBaseWheel();
  color("silver")
  translate([0, 0, 100 / 2])
  cube([$BaseW, $BaseL, 50], center = true);
}

///////////////////////////////////////////////////////////////////////////////////////
//SampleGrabberArm();
//SampleGrabberFrame();
//SampleGrabberLifterAttach();

//SampleGrabberMechanism($DoServo);

//color([0.7, 0, 0.2, 0.7])
//translate([-53, 0, 5])
//cube([38, 38, 89], center = true);

//ClimbTest();
ClimbTest2();