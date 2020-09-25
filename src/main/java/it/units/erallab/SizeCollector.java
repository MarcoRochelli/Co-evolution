package it.units.erallab;

import it.units.malelab.jgea.core.listener.Event;
import it.units.malelab.jgea.core.listener.collector.DataCollector;
import it.units.malelab.jgea.core.listener.collector.Item;

import java.util.Arrays;
import java.util.List;

public class SizeCollector implements DataCollector<Object, Object, Object> {

  int robotSize;
  // how do i get robot size????????


  public int getRobotSize() {
    return robotSize;
  }

  public void setRobotSize(int robotSize) {
    this.robotSize = robotSize;
  }

  @Override
  public List<Item> collect(Event<?, ?, ?> event) {
    return Arrays.asList(
        new Item("robot.size", robotSize, "%2d") // what is format??
    );
  }
}