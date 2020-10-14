package it.units.erallab;

import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
import org.apache.commons.lang3.SerializationUtils;

import java.util.Objects;
import java.util.Random;
import java.util.function.UnaryOperator;

public class ValidateRobot {

  public static UnaryOperator<Robot<?>> modifyRobot(int nOfModifications, Robot robot) {
    Random random = new Random();

    int numberOfVoxels = (int) robot.getVoxels().count(Objects::nonNull);
    int effectiveDamage = nOfModifications * numberOfVoxels / 100 * 15;

    Grid<SensingVoxel> body = robot.getVoxels();
    Grid<SensingVoxel> modifiedBody = robot.getVoxels();

    SensingVoxel sensingVoxel = null; // initializes sensing voxel copying a random voxel
    while (sensingVoxel == null) {
      int w = random.nextInt(robot.getVoxels().getW());
      int h = random.nextInt(robot.getVoxels().getH());
      sensingVoxel = body.get(w, h);
    }

    for (int damage = 1; damage < effectiveDamage; damage++) {
      boolean add = random.nextBoolean();
      int x = random.nextInt(robot.getVoxels().getW());
      int y = random.nextInt(robot.getVoxels().getH());

      // if i have to add and that voxel is empty add and increase damage
      if (add && modifiedBody.get(x, y) == null) {
        modifiedBody.set(x, y, SerializationUtils.clone(sensingVoxel));
        damage++;
      }
      // if i have to remove and that voxel is not empty try to remove
      else if (!add && modifiedBody.get(x, y) != null) {
        int previousSize = (int) modifiedBody.count(Objects::nonNull);
        modifiedBody.set(x, y, null); //remove a voxel

        // calculate new dimension of grid
        modifiedBody = Utils.gridLargestConnected(modifiedBody, Objects::nonNull);
        for (Grid.Entry<SensingVoxel> entry : modifiedBody) {
          if (entry.getValue() == null) {
            modifiedBody.set(entry.getX(), entry.getY(), null);
          }
        }
        int actualSize = (int) modifiedBody.count(Objects::nonNull);
        // if new grid is only one voxel smaller than previous one ok go on
        if (actualSize != previousSize - 1) {
          damage++;
        } else { // robot is broken in two so do not remove voxel
          modifiedBody.set(x, y, SerializationUtils.clone(sensingVoxel));
        }
      }
    }

    // return robot with the new body
    Grid<SensingVoxel> finalModifiedBody = modifiedBody;
    return newRobot -> new Robot<>(
        ((Robot<SensingVoxel>) robot).getController(),
        finalModifiedBody
        );
  }
}



