package frc.robot.autoController;

import java.util.ArrayList;
import java.util.List;

public class Path {
  private final List<Segment> segments;

  public Path(){
    segments = new ArrayList<Segment>();
  }

  public void addSegment(Segment segment){
    this.segments.add(segment);
  }
}
