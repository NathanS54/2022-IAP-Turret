// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;


  /** Creates a new DriveTrain. */
  public DriveTrain() {

    leftDriveTalon = newWPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = newWPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
  }



  private WPI_TalonSRX newWPI_TalonSRX(int leftdrivetalonport) {
    return null;
  }



  
  

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }
  @Override
  public void periodic() {
    tankDrive(RobotContainer.getJoy1().getY()*-0.2,RobotContainer.getJoy2().getY()*-0.2);
  }
}
