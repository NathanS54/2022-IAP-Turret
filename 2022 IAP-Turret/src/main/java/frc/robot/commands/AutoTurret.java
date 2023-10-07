
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.*;

public class AutoTurret extends CommandBase {
  /** Creates a new AutoTurret. */
  public final PhotonVision photon;
  public final Turret turret;
  public PIDController pid;
  public boolean directionToggle;
  public boolean manualToggle = true;
 
  private double limitSwitchClosed = 1.0;
  private double manualSwitchTime = 0.1;
  private Timer manualTimer;

  public AutoTurret(PhotonVision photon, Turret turret) {
    
    this.photon = photon;
    this.turret = turret;
    manualTimer = new Timer();
    pid = new PIDController(0.016, 0.0, 0.0005);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.resetEncoders();
    manualTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // When the trigger button is pressed, switch from manual to auto
    // when timer ends it will debounce
    if (RobotContainer.getJoy1().getRawButton(1) && manualTimer.get() >= manualSwitchTime) {
      manualToggle = !manualToggle;
      manualTimer.reset();
    }

    if (manualToggle) {
      // Uses slow, manual control by default
      turret.spin(-0.2*RobotContainer.getJoy1().getY());
    }
    if (!manualToggle) {
      // TODO - What if the target is in range, but
      // the turret can't spin there?
      if (!photon.targetExists()) {

        if (turret.getCCW_Reverse_LimitSw() == limitSwitchClosed) {
           // "True" means that it hit the limit switch
           // in the reverse direction, and will be going CW.
          directionToggle = true;
        }

        if (turret.getCW_Forward_LimitSw() == limitSwitchClosed) {
           // when false it hass hit the switch
           // when going forward it will beo going to CCW
          directionToggle = false;
        }

        if (directionToggle) { 
          // is CW when hit reverse
          turret.spin(0.3);
        } else if (!directionToggle) { // NOTE - Need to test directions
          // Goes CCW if hitting the forward limit switch
          turret.spin(-0.3);
        }

      // NOTE - Does nothing if the target exists, but the limit switches are hit.
      // This is a placeholder then for safety
      } else if (photon.targetExists() && !(turret.getCCW_Reverse_LimitSw() == limitSwitchClosed | turret.getCW_Forward_LimitSw() == limitSwitchClosed)) {
        double speed = pid.calculate(photon.getYaw()); // If it exists and within range, apply PID to the output
        turret.spin(speed); //when positive yaw value it should turn right (clockwise)
      } else {
        turret.spin(0.0); // wil disab,e the motor if all the conditions are met
      }

    }
    
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
