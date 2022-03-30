package frc.robot.Dashboard;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanisims.MkSwerveTrain;

public class Shuffle {

  private double lastTime = Timer.getFPGATimestamp();
  private boolean mUpdateDashboard;
  private int loopOverrunWarning;
  private int loopCounter;

  public static Shuffle getInstance() {
    return InstanceHolder.mInstance;
  }

  public void updateDeltaTime() {
    loopCounter++;
    double time = Timer.getFPGATimestamp();
    double dt = (time - lastTime) * 1e3;
    SmartDashboard.putNumber("Loop Dt", dt);
    lastTime = time;

    if (dt > 22) {
      loopOverrunWarning++;
    }

    if (loopCounter == 500) {
      if (loopOverrunWarning > 10) {
        Shuffleboard.addEventMarker("Loop Time Over 22ms for more than 10 loops in the past 5 seconds.", EventImportance.kHigh);
        loopCounter = 0;
      }
    }
  }

  public void update() {
    updateDeltaTime();
    if (mUpdateDashboard) {
      mUpdateDashboard = false;
      //put update functions here
    } else {
      mUpdateDashboard = true;
    }
  }

  public void startWidgets()
  {
    Shuffleboard.getTab("ShuffleBoard")
    .add("ff", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 220000)) // specify widget properties here
    .getEntry();
  }

  public void updateValues()
  {
    SmartDashboard.putNumber("ff", MkSwerveTrain.getInstance().getModules()[0].getFF());
  }

  private static class InstanceHolder {

    private static final Shuffle mInstance = new Shuffle();
  }
}
