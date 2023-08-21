// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.health;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import java.nio.file.Files;
import java.nio.file.Path;
import org.littletonrobotics.junction.Logger;

public class HealthSubsystem extends LifecycleSubsystem {
  private static final Path TEST_PATH = Path.of("/var", "tmp");

  private static boolean isDiskHealthy() {
    return Files.exists(TEST_PATH);
  }

  public HealthSubsystem() {
    super(SubsystemPriority.HEALTH);
  }

  @Override
  public void robotPeriodic() {
    boolean diskHealthy = isDiskHealthy();
    Logger.getInstance().recordOutput("Health/DiskHealthy", diskHealthy);
    SmartDashboard.putBoolean("Disk healthy", diskHealthy);
  }
}
