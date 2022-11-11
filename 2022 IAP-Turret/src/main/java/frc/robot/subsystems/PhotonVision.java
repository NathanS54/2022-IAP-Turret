// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  
    
  private PhotonCamera camera;
  private PhotonTrackedTarget target;

  private boolean hasTarget;
  private double yaw;
  private double pitch;
  private double area;
  private double skew;
  
  public PhotonVision() {
    PhotonCamera camera = new PhotonCamera("photovision");
    camera.setDriverMode(true);
    camera.setPipelineIndex(2);
  }

   public boolean targetExists(){
     return hasTarget;
   }
    
   public double getYaw(){
     return yaw;
   }

   public double getPitch(){
      return pitch;
    
   }
    
   public double getArea(){
     return area;
   }
    
   public double getSkew(){
     return skew;
   }

    
    
    



  

  @Override
  public void periodic() {
    var result = camera.getLatestResult();
     hasTarget = result.hasTargets();
    if (hasTarget){
      target = result.getBestTarget();
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      skew = target.getSkew();
    }
     SmartDashboard.putNumber("Yaw,", yaw);
     SmartDashboard.putNumber("Pitch",pitch);
     SmartDashboard.putNumber("Area",area);
     SmartDashboard.putNumber("Skew", skew);
  }
}
