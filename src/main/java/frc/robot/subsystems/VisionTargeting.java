// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionTargeting extends SubsystemBase
{
  private static double tv;
  private double ty;
  private double tx; 
  private String tclass;
  private double distance;
  /** Creates a new VisionTargeting. */
  public VisionTargeting() 
  {

  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-b");
    NetworkTableEntry networkTableTx = table.getEntry("tx");
    NetworkTableEntry networkTableTv = table.getEntry("tv");
    NetworkTableEntry networkTableTy = table.getEntry("ty");
    NetworkTableEntry networkTableTclass = table.getEntry("tclass");
    NetworkTableEntry networkTablebotpose_targetspace = table.getEntry("botpose_targetspace");
   
    //read values periodically
    tx = networkTableTx.getDouble(0.0);
    tv = networkTableTv.getDouble(0.0);
    ty = networkTableTy.getDouble(0.0);
    tclass = networkTableTclass.getString("");
    double botpose[] = networkTablebotpose_targetspace.getDoubleArray(new double[6]);
    distance = botpose[2];
  }
  
  public static boolean getTargetVisible()
  {
    if (tv == 1.0)
    {
      return true;
    }
    else 
    {
      return false;
    }
  }

  public double getTy()
  {
    return ty;
  }

  public double getTx()
  {
    return tx;
  }

  public double getDistance()
  {
    return distance;
  }

  public double getDistanceToTarget(boolean isAprilTag)
  {
    double targetHeight = 0;
    double tyRadians = (3.14159 / 180.0) * getTy();
    double distanceToTarget = (targetHeight - Constants.VisionTargeting.cameraHeight) / Math.tan(tyRadians);

    if(VisionTargeting.getTargetVisible()) 
    {
      if(isAprilTag) 
      {
        targetHeight = Constants.VisionTargeting.aprilTagHeight;
        SmartDashboard.putNumber("distance to target", distanceToTarget);
        return distanceToTarget;
      }

      else 
      {
        if(getObjectType() == "cube")
        {
          targetHeight = Constants.VisionTargeting.cubeHeight;
        }
    
        if(getObjectType() == "cone")
        {
          targetHeight = Constants.VisionTargeting.coneHeight;
        }
        SmartDashboard.putNumber("distance to target", distanceToTarget);
        return distanceToTarget;
      }
    }

    else 
    {
      SmartDashboard.putNumber("distance to target", distanceToTarget);
      return 0;
    }
  }  

  public String getObjectType()
  {
      if (tclass.startsWith("cube"))
      {
        return "cube";
      }
      if (tclass.startsWith("cone"))
      {
        return "cone";
      }
      return null;
  }

  public void setPipeline(int pipeline)
  {
    NetworkTableInstance.getDefault().getTable("limelight-b").getEntry("pipeline").setNumber(pipeline);
  }

  public String getTclass() {
    return tclass;
  }
}
