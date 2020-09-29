package it.units.erallab;

import it.units.erallab.hmsrobots.core.objects.Voxel;
import it.units.erallab.hmsrobots.core.sensors.Sensor;

public class Constant implements Sensor {
  private final double[] values;
  private final Domain[] domains;

  public Constant(double[] values, Domain[] domains) {
    if (values.length != domains.length) {
      throw new IllegalArgumentException("values and domains lengths are different");
    }
    this.values = values;
    this.domains = domains;
  }

  @Override
  public Domain[] domains() {
    return domains;
  }

  @Override
  public double[] sense(Voxel voxel, double t) {
    return values;
  }
}
