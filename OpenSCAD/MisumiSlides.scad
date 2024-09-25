

module RoundedBlockV2($XDim = 10, $YDim = 10, $ZDim = 10, $D1 = 0, $D2 = 0, $D3 = 0, $D4 = 0, $D5 = 0, $D6 = 0, $D7 = 0, $D8 = 0)
{
  $fn = 20;
  hull()
  {
    translate([($XDim - $D1) / 2, ($YDim - $D1) / 2, ($ZDim - $D1)/ 2])
      sphere(d = $D1);
    translate([-($XDim - $D2) / 2, ($YDim - $D2) / 2, ($ZDim - $D2)/ 2])
      sphere(d = $D2);
    translate([-($XDim - $D3) / 2, -($YDim - $D3) / 2, ($ZDim - $D3)/ 2])
      sphere(d = $D3);
    translate([($XDim - $D4) / 2, -($YDim - $D4) / 2, ($ZDim - $D4)/ 2])
      sphere(d = $D4);
    
    translate([($XDim - $D5) / 2, ($YDim - $D5) / 2, -($ZDim - $D5)/ 2])
      sphere(d = $D5);
    translate([-($XDim - $D6) / 2, ($YDim - $D6) / 2, -($ZDim - $D6)/ 2])
      sphere(d = $D6);
    translate([-($XDim - $D7) / 2, -($YDim - $D7) / 2, -($ZDim - $D7)/ 2])
      sphere(d = $D7);
    translate([($XDim - $D8) / 2, -($YDim - $D8) / 2, -($ZDim - $D8)/ 2])
      sphere(d = $D8);
    
  }
}
module MisumiLiftSlideDrillTemplate2($Stages = 1)
{
  $fn = 20;
  $Clearance = 0.2;
  $YOffset = 5 + ($Clearance / 2);
  $XOffset = 12;//
  $XHoleSpacing = 33;
  $YSpacing = 16;
  
  difference()
  {
    RoundedBlockV2($XDim = 53, $YDim = ($Stages * $YSpacing) + 6, $ZDim = 27, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    translate([-2, 0, 0])
    {
      cube([53, ($Stages * $YSpacing) + $Clearance, 20.2], center = true);
      for (i = [0:$Stages - 1])
      {
        translate([(53 / 2) - $XOffset, (($Stages * $YSpacing) / 2) - $YOffset - (i * $YSpacing), 0])
        {
          cylinder(d = 2, h = 30, center = true);
            translate([-$XHoleSpacing, 0, 0])
              cylinder(d = 2, h = 30, center = true);
        }
      }
    }
  }
  
}

module MisumiLiftSlideDrillTemplate1()
{
  difference()
  {
    union()
    {
      difference()
      {
        union()
        {
          //Main body
          translate([0, 0, 0])
            RoundedBlockV2($XDim = 53, $YDim = 28, $ZDim = 17, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
        }
        //Double slide coupling
        translate([- 4, 0, 0])    
          cube([60 - 8, 20.5, 15.4], center = true);
        translate([- 4, 0, 0])    
          cube([60 - 8, 19.5, 20], center = true);
        //Slide mount holes
        translate([10, 0, (16 / 2) - 3])
          rotate(90, [1, 0, 0])
            cylinder(d = 2, h = 50, $fn = 30, center = true);
        translate([-23, 0, (16 / 2) - 3])
          rotate(90, [1, 0, 0])
            cylinder(d = 2, h = 50, $fn = 30, center = true);
      }
    }
  }
}

module MisumiLiftSlideInnerFirst($Multi = false, $Bottom = 0)
{
  difference()
  {
    union()
    {
      difference()
      {
        union()
        {
          //Main body
          translate([2.75, 0, 0])
            RoundedBlockV2($XDim = 65.5, $YDim = 28, $ZDim = 17, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
          //Pulley block
          translate([30 - 5, -16, 2])
            RoundedBlockV2($XDim = 21, $YDim = 8 + 6, $ZDim = 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
        }
        //Double slide coupling
        translate([- 4, 0, 0])    
          cube([60 - 8, 20.5, 15.4], center = true);
        translate([- 4, 0, 0])    
          cube([60 - 8, 19.5, 20], center = true);
        //Inner carrier clearence
        translate([0, -13/2, -9])
          cube([40, 13, 8]);
        //Pulley opening
        translate([30 - 5, (-(30 + 4) / 2), 2])
          cube([13, 4.5, 10], center = true);
        //Slide mount holes
        mirror([0, 0, $Bottom])
        {
          translate([10, 0, (16 / 2) - 3])
            rotate(90, [1, 0, 0])
              cylinder(d = 3, h = 50, $fn = 30, center = true);
          translate([-23, 0, (16 / 2) - 3])
            rotate(90, [1, 0, 0])
              cylinder(d = 3, h = 50, $fn = 30, center = true);
        }
      }
      //Alignment guide
      difference()
      {
        translate([15.3875, 0, 0])
          rotate(90, [0, 1, 0])
            cylinder(d = 7.3, h = 20.1);
        translate([15.4, 0, 0])
          cube([13.2, 9, 3], center = true);
      }
    }
    //Bearing shaft
    translate([30 - 5, 0, 2])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30, center = true);
    
    if ($Multi)
    {
      mirror([0, 0, $Bottom])
        translate([-40, -20, -10 - (17 / 2) + 2.5])
          cube([80, 40, 10]);
    }
  }
}

module MisumiLiftSlideSpringMount($MountD = 2.9)
{
  $fn = 20;
  $WidthAdder = 10;
  difference()
  {
    union()
    {
      //Main block
      translate([90 / 2, 28 / 2 , (8 - $WidthAdder) / 2])
        RoundedBlockV2($XDim = 90, $YDim = 28, $ZDim = 8 + $WidthAdder, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Attach/pulley block
      translate([21 / 2, -(8 - 6) / 2, (8 + 6.5) / 2])
        RoundedBlockV2($XDim = 21, $YDim = 8 + 6, $ZDim = 8 + 6.5, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Spring blocker
      translate([24, -(8 - 6) / 2, - 6 + ($WidthAdder) / 2])
        RoundedBlockV2($XDim = 10, $YDim = 8 + 6, $ZDim = 8 + $WidthAdder, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    }
    //Slider alignment groove
    translate([0, (28 - 20.2) / 2, 7])
      cube([90, 20.2, 10]);
    //Bearing opening
    translate([21/2, -3, 19/2])
      cube([13, 4.5, 20], center = true);
    //Bearing shaft
    translate([21 / 2, 32, 5])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Bearing cord guide
    translate([(21/2) + 9, -3, 13.5])
      cube([10, 2, 2], center = true);
    //Spring mount holes
    translate([35 + (10 / 2) + 45, 32, -6])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    translate([35 + (10 / 2) + 45, 32, -0.5])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Spring cord opening2
    translate([25, -5, -1])
      cube([20, 10, 2], center = true);
    translate([25, -5, -6])
      cube([20, 10, 2], center = true);
    //Slider attach hole options
    //Pulley block flush with top
    translate([14.5, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Pulley flush with top
    translate([14.5 + 12, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Full travel clearance
    translate([14.5 + 12 + 23, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Return clearance location 1
    translate([14.5 + 12 + 8, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Return clearance location 2
    translate([14.5 + 12 + 15, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
  }
}

module MisumiLiftSlideReturn($MountD = 2.9, $Stages = 3)
{
  difference()
  {
    union()
    {
      //Main block
      translate([90 / 2, -28 / 2, 8 / 2])
        RoundedBlockV2($XDim = 90, $YDim = 28, $ZDim = 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Return support block
      translate([21 / 2, -28 / 2, ((8 + ($Stages * 16) + 5) / 2)])
        RoundedBlockV2($XDim = 21, $YDim = 28, $ZDim = 8 + ($Stages * 16) + 5, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Pulley block
      translate([21 / 2, (8 - 6) / 2, 8 / 2])
        RoundedBlockV2($XDim = 21, $YDim = 8 + 6, $ZDim = 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Pulley return block
      translate([21 / 2, (8 - 6) / 2, (8 / 2) + (16 * $Stages) + 5])
        RoundedBlockV2($XDim = 21, $YDim = 8 + 6, $ZDim = 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    }
    //Slider alignment groove
    translate([21, -(28 + 20.5) / 2, 7])
      cube([90, 20.5, 10]);
    //Bearing opening
    translate([21/2, 3, 9/2])
      cube([13, 4.5, 10], center = true);
    //Return bearing opening
    translate([21/2, 3, (9/2) + (16 * $Stages) + 5])
      cube([13, 4.5, 10], center = true);
    //Bearing shaft
    translate([21 / 2, 15, 5])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Return bearing shaft
    translate([21 / 2, 15, 5  + (16 * $Stages) + 5])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Mount hole
    translate([14.5 + 12 + 8 + 1.5, -28 / 2, -1])
      cylinder(d = $MountD, h = 20);
  }
}

module MisumiLiftSlideOuterFirst($MountD = 2.9)
{
  difference()
  {
    union()
    {
      translate([90 / 2, 28 / 2, 8 / 2])
        RoundedBlockV2($XDim = 90, $YDim = 28, $ZDim = 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      translate([21 / 2, -(8 - 6) / 2, 8 / 2])
        RoundedBlockV2($XDim = 21, $YDim = 8 + 6, $ZDim = 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    }
    //Slider alignment groove
    translate([0, (28 - 20.2) / 2, 7])
      cube([90, 20.2, 10]);
    //Bearing opening
    translate([21/2, -3, 9/2])
      cube([13, 4.5, 10], center = true);
    translate([21 / 2, 32, 5])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Slider attach hole options
    //Pulley block flush with top
    translate([14.5, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
    //Pulley flush with top
    translate([14.5 + 12, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
    //Full travel clearance
    translate([14.5 + 12 + 23, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
    //Return clearance location 1
    translate([14.5 + 12 + 8, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
    //Return clearance location 2
    translate([14.5 + 12 + 15, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
  }
}

module MisumiSliderSet($Sliders = 3)
{
  //******************************************
  //******************************************
  //Misumi slider kits
  //
  //Dual sliders need
  //    1x left (non-mirrored)
  //    1x right
  //    1x spring mount
  //    1x top inner
  //    1x bottom inner
  //Tripple sliders need
  //    1x left (non-mirrored)
  //    1x right
  //    1x spring mount
  //    2x top inner
  //    2x bottom inner
  //******************************************
  //******************************************
  
  translate([-8, 0, 0])
    mirror([1, 0, 0])
      MisumiLiftSlideOuterFirst();
  MisumiLiftSlideOuterFirst();
  translate([60, 33, 0])
    rotate(90, [0, 0, 1])
      MisumiLiftSlideReturn($MountD = 2.9);
  for ($i = [1:$Sliders - 1])
  {
    translate([10, ($i * 40) + 19, 8.5])
      rotate(180, [1, 0, 0])
        MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 0);//Top inner pulley
    translate([-10, ($i * 40) + 13, 6.0124])
      rotate(180, [0, 1, 0])
        MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 1);//Bottom inner pulley
    translate([-60, 33, 10])
      rotate(90, [0, 0, 1])
        MisumiLiftSlideSpringMount($MountD = 2.9);
  }
}

module MisumiRailSet(support1 = true, length = 300, stages = 2, position = 100, offset = 0, motorposition, channelholes, offsetholes, dopulleyguide)
{
  hoffset = support1 ? 15 : 0;
  //Return style 0 = Return cord lines up with edge of outer slide
  //Return style 1 = Return cord lines up with pull up pulley
//  returnextension = (returnstyle == 0) ? ((15/2) + (stages * 16) + (hoffset * (stages - 1)) - 3) :
//                                         ((15/2) + (stages * 16) + (hoffset * (stages - 1)) - 13);
 
  for (i = [0:(stages - 1)])
  {
    translate([0, (i * (16 + 15)) + hoffset - 15, (i * offset * 2) + (i * (position / stages))])
    {
      if (!(!support1 && (i == 0)))
        translate([0, (15/2), 0])
          Extrusion15mm(length = length);
      if (i == stages - 1)
        translate([0, 15 , offset])
          MisumiSlide(length = length, position = (position / stages), showupper = true, showlower = true, pulleyextension = 5, showpulley = true, offset = offset);
      else
        translate([0, 15 , offset])
          MisumiSlide(length = length, position = (position / stages), showupper = true, showlower = true, pulleyextension = 0, showpulley = true, offset = offset);
    }
  }
  //Pulley return plate
  //Need to update this to new return pulley
  /*
  translate([10, (15/2), -12 + offset])
    MisumiPulleyReturnPlate(showpulley = true, extension = returnextension, holespacing = 16, voffset = 15, width = width);
  if (dopulleyguide)
  {
    translate([10, 15/2, 185])
      rotate(90, [1, 0, 0])
        SlidePulleyGuide(width);
  }
  */
  
  //Motor mount
  if (motorposition != -1)
  {
    translate([-5, -20, motorposition])
      rotate(90, [0, 1,0])
        MotorAndFrame(ChannelHoles = channelholes, Rx = -1, Ry = 0, Rz = -1, OffsetHoles = offsetholes);
  }
}

module MisumiSlideRail(length = 300)
{
  difference()
  {
    translate([-10, 0, 0])
      cube([20, 7.2, length]);
    translate([-8, 2.01, -0.5])
      cube([16, 5.2, length + 1]);
    rotate(-90, [1, 0, 0])
    {
      translate([0, -15, -0.1])
        cylinder(d = 3.1, h = 10);
      translate([0, -length + 15, -0.1])
        cylinder(d = 3.1, h = 10);
      translate([0, -length / 2, -0.1])
        cylinder(d = 3.1, h = 10);
    }
  }
}
 
module Extrusion15mm(length = 300)
{
  color([0.95, 0.9, 0.95])
    translate([0, 0, length / 2])
      cube([15, 15, length], center = true);
  color([.8, .8, .8])
  {
    translate([0, 0, length / 2])
      cube([3.001, 15.001, length + 0.001], center = true);
    translate([0, 0, length / 2])
      cube([15.001, 3.001, length + 0.001], center = true);
  }
}

module MotorAndFrame(ChannelHoles, Rx, Ry, Rz, OffsetHoles = 0)
{
  //Motor
  cylinder(d = 36, h = 117);
  //Motor shaft
  translate([0, 0, -20])
    color("silver")
      cylinder(d = 8, h = 20, $fn = 6);
  //Need to update
  /*
  if (ChannelHoles > 0)
  {
    translate([0, 0, - (24 * OffsetHoles)])
      CChannelCentered(Holes = ChannelHoles, Depth = 48, Rx = Rx, Ry = Ry, Rz = Rz);
  }
  */
}

module MisumiSlide(length, position, showupper = true, showlower = true, pulleyextension, showpulley = true, offset)
{
  color("silver")
  {
    MisumiSlideRail(length = length);
    translate([0, 16, position])
      mirror([0, 1, 0])
        MisumiSlideRail(length = length);
  }
  color("darkgray")
    translate([-8, 2, position / 2])
      cube([16, 12, length]);   
  //Need to update these to new pulley guides
}


//******************************************
//******************************************
//Misumi slider kits
//
//Dual sliders need
//    2x left (non-mirrored)
//    1x right
//    1x top inner
//    1x bottom inner
//Tripple sliders need
//    2x left (non-mirrored)
//    1x right
//    2x top inner
//    2x bottom inner
//mirror([1, 0, 0])
//  MisumiLiftSlideOuterFirst();
//MisumiLiftSlideOuterFirst();
//MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 0);//Top inner pulley
//MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 1);//Bottom inner pulley
//******************************************
//******************************************
//MisumiSliderSet($Sliders = 3);
//MisumiLiftSlideSpringMount($MountD = 2.9);
//MisumiLiftSlideReturn($Stages = 3, $MountD = 2.9);
//MisumiLiftSlideDrillTemplate1();
MisumiLiftSlideDrillTemplate2($Stages = 3);
//MisumiRailSet(support1 = true, length = 300, stages = 2, position = 100, offset = 0, motorposition = 40, channelholes = 3, offsetholes = 10, dopulleyguide = true);
