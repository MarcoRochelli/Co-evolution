package it.units.erallab;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.AreaRatio;
import it.units.erallab.hmsrobots.core.sensors.Normalization;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.core.sensors.Velocity;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.lang3.SerializationUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;

import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Function;

public class DoubleMapper implements Function<List<Double>, Robot<?>> {

  private final boolean heterogeneous;
  private final int width;
  private final int height;

  private final List<Sensor> sensors;
  private final int[] innerNeurons;
  private final int signals;
  private final static double THRESHOLD = 0d;

  public DoubleMapper(boolean heterogeneous, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
    this.heterogeneous = heterogeneous;
    this.width = width;
    this.height = height;
    this.sensors = sensors;
    this.innerNeurons = innerNeurons;
    this.signals = signals;
  }

  public int getGenotypeSize() {  // returns expected length of the genotype
    int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum();
    int nOfOutputs = signals * 4 + 1;
    int nOfWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));

    if (heterogeneous) {
      return width * height + nOfWeights * width * height;
    } else

      return width * height + nOfWeights;
  }

  @Override
  public Robot<?> apply(List<Double> genotype) { //  is the genotype

    int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum();
    int nOfOutputs = signals * 4 + 1;
    int nOfVoxelWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));
    if (genotype.size() != getGenotypeSize()) { // correct length of the genotype
      throw new IllegalArgumentException("Sensor list has wrong dimension");
    }

    SensingVoxel sensingVoxel = new SensingVoxel(sensors);

    int c = 0;
    Grid<SensingVoxel> body = Grid.create(width, height);
    for (double entry : genotype) {
      if (c < width * height) {  // < or <= ? < is correct
        if (entry > THRESHOLD) {
          body.set(c % width, c / width, SerializationUtils.clone(sensingVoxel));
        } else {
          body.set(c % width, c / width, null);
        }
        c = c + 1;
      }
    }

    // creates an empty grid
    Grid<SensingVoxel> emptyBody = Grid.create(body.getW(), body.getH(), (x, y) -> null);
    // checks if the robot is empty
    if (body.equals(emptyBody)) {
      body.set(0, 0, SerializationUtils.clone(sensingVoxel)); // if the body is empty puts a voxel (0,0)
    }

    // checks if the robot is connected
    body = Utils.gridLargestConnected(body, Objects::nonNull);
    for (Grid.Entry<SensingVoxel> entry : body) {
      if (entry.getValue() == null) {
        body.set(entry.getX(), entry.getY(), null);
      }
    }

    /*
    // to know number of effective voxel of the robot effective width and effective height  WORKS
    int nOfVoxels = 0;
    for (Grid.Entry<SensingVoxel> entry : body) {
      if (entry.getValue() != null) {
        nOfVoxels++;
      }
    }
    Grid<SensingVoxel> croppedBody = Utils.cropGrid(body,Objects::nonNull);
    int effectiveWidth = croppedBody.getW();
    int effectiveHeight = croppedBody.getH();
    System.out.println("dimensione del robot: " + nOfVoxels);
    System.out.println("larghezza del robot: " + effectiveWidth);
    System.out.println("altezza del robot: " + effectiveHeight);
     */

    // creates a distributed controller
    DistributedSensing distributedSensing = new DistributedSensing(body, signals);
    for (Grid.Entry<SensingVoxel> entry : body) {
      MultiLayerPerceptron mlp = new MultiLayerPerceptron(
          MultiLayerPerceptron.ActivationFunction.TANH,
          nOfInputs,
          innerNeurons,
          nOfOutputs
      );

      // creates an array of doubles from the list of the genotype
      List<Double> w = genotype.subList(width * height, genotype.size());
      double[] weights = new double[w.size()];
      for (int i = 0; i < weights.length; i++) {
        weights[i] = w.get(i);
      }

      if (heterogeneous) {
        int from = entry.getX() * nOfVoxelWeights + entry.getY() * width * nOfVoxelWeights; //  + width * height is not needed beacause i remove it before
        int to = from + nOfVoxelWeights;
        double[] voxelWeights = Arrays.copyOfRange(weights, from, to);
        /*
        System.out.println("parto da: " + from);
        System.out.println("fino a: " + to);
        for (int i = 0; i < voxelWeights.length; i++) {
          System.out.println("voxelWeights:[" + i + "] " + voxelWeights[i]);
        }

         */
        mlp.setParams(voxelWeights);
      } else {
        // i have to assign the correct subset of weights to this
        mlp.setParams(weights);
      }
      distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp);
    }

    return new Robot<>(
        distributedSensing,
        body //SerializationUtils.clone(body) i think it is better not to copy this
    );
  }


  // to test the mapper
  public static void main(String[] args) {
         /* // how to create a sensing voxel
        SensingVoxel sensingVoxel = new SensingVoxel(List.of(  // pass list of sensors to put into the voxel
                new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
                new Normalization(new AreaRatio())
        ));
         */

    Random random = new Random();
    // problem to solve
    Locomotion locomotion = new Locomotion(
        40,
        Locomotion.createTerrain("flat"), // se uneven deve esserci qualcosa dopo es. uneven5; uneven-qualcosa da errore
        Lists.newArrayList(Locomotion.Metric.TRAVEL_X_VELOCITY),
        new Settings()
    );

    List<Sensor> sensors = List.of(  // list of sensors to use
        new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
        new Normalization(new AreaRatio())
    );
    int[] innerNeurons = new int[0]; // if more than 0 gives error: not enough heap memory
    for (int i = 0; i < innerNeurons.length; i++) {
      innerNeurons[i] = random.nextInt();
    }

    DoubleMapper mapper = new DoubleMapper(true, 10, 10, sensors, innerNeurons, 1);
    UniformDoubleFactory udf = new UniformDoubleFactory(-1, 1);
    System.out.println("lunghezza genotipo: " + mapper.getGenotypeSize()); // to know genotype size
    FixedLengthListFactory<Double> factory = new FixedLengthListFactory<>(mapper.getGenotypeSize(), udf);
    List<Double> genotype = factory.build(random);
    Robot<?> robot = mapper.apply(genotype);


    Grid<Pair<String, Robot<?>>> namedSolutionGrid = Grid.create(1, 1);// i create a one for one graphic
    namedSolutionGrid.set(0, 0, Pair.of("mostro", robot));        // where i put mostro
    ScheduledExecutorService uiExecutor = Executors.newScheduledThreadPool(4); // number of available core??
    ExecutorService executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    GridOnlineViewer gridOnlineViewer = new GridOnlineViewer(Grid.create(namedSolutionGrid, Pair::getLeft), uiExecutor);
    gridOnlineViewer.start(1); // delay before showing the video simulation
    GridEpisodeRunner<Robot<?>> runner = new GridEpisodeRunner<>(
        namedSolutionGrid,
        locomotion,
        gridOnlineViewer,
        executor
    );
    runner.run();
  }
}
