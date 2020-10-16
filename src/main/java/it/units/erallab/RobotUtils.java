package it.units.erallab;

import it.units.erallab.hmsrobots.core.controllers.PhaseSin;
import it.units.erallab.hmsrobots.core.objects.ControllableVoxel;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
import org.apache.commons.lang3.SerializationUtils;

import java.util.Objects;
import java.util.Random;
import java.util.function.UnaryOperator;

public class RobotUtils {

  public static <V extends ControllableVoxel> Robot<V> modifyRobot(Robot<V> robot, int nOfModifications, long seed) {
    Random random = new Random(seed);
    int numberOfVoxels = (int) robot.getVoxels().count(Objects::nonNull);
    int effectiveDamage = (int) (nOfModifications * ((double) numberOfVoxels / 100d * 10d)); // take care i forgot type cast and was integer division

    Grid<? extends V> body = robot.getVoxels();
    Grid<V> modifiedBody = Grid.create(body, v -> v);
    V sensingVoxel = body.values().stream().filter(Objects::nonNull).findFirst().get(); // initializes sensing voxel copying a random voxel

    int nOfEffectiveModifications = 0;
    while (nOfEffectiveModifications < effectiveDamage) {
      boolean add = random.nextBoolean();
      int x = random.nextInt(robot.getVoxels().getW());
      int y = random.nextInt(robot.getVoxels().getH());

      // if i have to add and that voxel is empty add and increase damage
      if (add && modifiedBody.get(x, y) == null) {
        modifiedBody.set(x, y, SerializationUtils.clone(sensingVoxel));
        nOfEffectiveModifications++;
      }
      // if i have to remove and that voxel is not empty try to remove
      else if (!add && modifiedBody.get(x, y) != null) {
        int previousSize = (int) modifiedBody.count(Objects::nonNull);
        modifiedBody.set(x, y, null); //remove a voxel

        // calculate new dimension of grid
        modifiedBody = Utils.gridLargestConnected(modifiedBody, Objects::nonNull);

        int currentSize = (int) modifiedBody.count(Objects::nonNull);
        // if new grid is only one voxel smaller than previous one ok go on
        if (currentSize == previousSize - 1) {
          nOfEffectiveModifications++;
        } else { // robot is broken in two so do not remove voxel
          modifiedBody.set(x, y, SerializationUtils.clone(sensingVoxel));
        }
      }
    }
    // return robot with the new body
    return new Robot<V>(
        robot.getController(), // check also grid of controller
        modifiedBody // maybe i have to write here a lambda or something  but how??  PROBLEM IS HERE
    );
  }

  public static void main(String[] args) {

    Robot<ControllableVoxel> robot = new Robot<>(
        new PhaseSin(1, 1, Grid.create(5, 5, 0d)),
        Grid.create(5, 5, (x, y) -> (x == 3) ? new ControllableVoxel() : null)
    );

    modifyRobot(robot, 4, 1);

  }

}
