// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionTargeting extends SubsystemBase 
{
  private double tv;
  private double ty;
  private double tx; 
  private String tclass;
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
   
    //read values periodically
    tx = networkTableTx.getDouble(0.0);
    tv = networkTableTv.getDouble(0.0);
    ty = networkTableTy.getDouble(0.0);
    tclass = networkTableTclass.getString("");
  }
  
  public double getTv()
  {
    return tv;
  }

  public double getTy()
  {
    return ty;
  }

  public double getTx()
  {
    return tx;
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
}
