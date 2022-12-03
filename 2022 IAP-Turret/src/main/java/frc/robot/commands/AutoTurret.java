// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Turret;

public class AutoTurret<Timer> extends CommandBase {
  
  /** Creates a new AutoTurret. */
  public final PhotonVision photon;
  public final Turret turret;
  public PIDController pid;
  public boolean directionToggle;
  public boolean manualToggle = true;
  // The constant at which the limit switch return ta physically closed rate
  private double limitSwitchClosed = 0.0; // 0 for n1ormally closed
  private double manualSwitchTime = 0.1; // actual seconds Irl
  

  public AutoTurret(PhotonVision photon, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photon = photon;
    this.turret = turret;
    pid = new PIDController(0.0,0.0,0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
