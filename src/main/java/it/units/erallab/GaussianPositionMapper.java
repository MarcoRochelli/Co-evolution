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
import java.util.stream.Collectors;
import java.util.stream.Stream;

import static java.lang.Math.*;

public class GaussianPositionMapper implements Function<List<Double>, Robot<?>> {


  private final boolean heterogeneous;
  private final int nOfGaussians;
  private final int width;
  private final int height;

  private final List<Sensor> sensors;
  private final boolean hasPositionSensor;

  private final int[] innerNeurons;
  private final int signals;

  private final static double THRESHOLD = 0d;


  public GaussianPositionMapper(boolean heterogeneous, int nOfGaussians, int width, int height, List<Sensor> sensors, boolean hasPositionSensor, int[] innerNeurons, int signals) {
    this.heterogeneous = heterogeneous;
    this.nOfGaussians = nOfGaussians;
    this.width = width;
    this.height = height;
    this.sensors = sensors;
    this.hasPositionSensor = hasPositionSensor;
    this.innerNeurons = innerNeurons;
    this.signals = signals;
  }

  public int getGenotypeSize() {  // returns expected length of the genotype
    int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum() + (hasPositionSensor ? 2 : 0); // +2 are inputs of position if it is true
    int nOfOutputs = signals * 4 + 1;
    int nOfWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));
    int nOfBodyParams = 5 * nOfGaussians;
    if (heterogeneous) {
      return nOfBodyParams + nOfWeights * width * height;
    } else
      return nOfBodyParams + nOfWeights;
  }

  @Override
  public Robot<?> apply(List<Double> genotype) {
    int nOfBodyParams = 5 * nOfGaussians;
    int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum()  + (hasPositionSensor ? 2 : 0);
    int nOfOutputs = signals * 4 + 1;
    int nOfVoxelWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));
    if (genotype.size() != getGenotypeSize()) {
      throw new IllegalArgumentException("Sensor list has wrong dimension");
    }


    // CREATES A GRID THAT IS SUM OF THE GAUSSIANS
    Grid<Double> values = Grid.create(width, height);
    for (Grid.Entry<Double> entry : values) {
      double value = 0;
      for (int i = 0; i < nOfGaussians; i++) {
        double weight = genotype.get(5 * i);
        double Ux = genotype.get(1 + 5 * i);
        double Uy = genotype.get(2 + 5 * i);
        double Ox = genotype.get(3 + 5 * i);
        double Oy = genotype.get(4 + 5 * i);
        // normalization
        double x = (double) entry.getX() / (double) width;
        double y = (double) entry.getY() / (double) height;
        value += weight * (exp(-0.5 * (pow((x - Ux), 2.0) / pow(Ox, 2.0) + pow((y - Uy), 2.0) / pow(Oy, 2.0))) / (2 * PI * Ox * Oy));
      }
      values.set(entry.getX(), entry.getY(), value);
    }

    int c = 0;
    Grid<Boolean> shape = Grid.create(width, height);
    for (Grid.Entry<Double> entry : values) {
      if (c < width * height) {  // < or <= ? < is correct
        if (entry.getValue() > THRESHOLD) {
          shape.set(c % width, c / width, true);
        } else {
          shape.set(c % width, c / width, false);
        }
        c = c + 1;
      }
    }
    if (shape.values().stream().noneMatch(b -> b)) { // better way to check if the grid is empty
      shape = Grid.create(1, 1, true);
    }

    Grid<SensingVoxel> body = Grid.create(width, height,
        (x, y) -> new SensingVoxel(
            Stream.concat(
                sensors.stream().map(SerializationUtils::clone),
                List.of(
                    new Constant( // i create a new sensor
                        new double[]{(double)x / (double)width, (double)y / (double)height}, // and i pass it its normalized position
                        new Sensor.Domain[]{Sensor.Domain.of(0, 1), Sensor.Domain.of(0, 1)}
                    )
                ).stream()
            ).collect(Collectors.toList())
        )
    );

    for (Grid.Entry<Boolean> entry : shape){ // links shape with body
      if (!entry.getValue()){
        body.set(entry.getX(), entry.getY(), null);
      }
    }

    // checks if the robot is connected
    body = Utils.gridLargestConnected(body, Objects::nonNull);
    for (Grid.Entry<SensingVoxel> entry : body) {
      if (entry.getValue() == null) {
        body.set(entry.getX(), entry.getY(), null);
      }
    }

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
      List<Double> w = genotype.subList(nOfBodyParams, genotype.size());
      double[] weights = new double[w.size()];
      for (int i = 0; i < weights.length; i++) {
        weights[i] = w.get(i);
      }

      if (heterogeneous) {
        int from = entry.getX() * nOfVoxelWeights + entry.getY() * width * nOfVoxelWeights; //  + width * height is not needed because i removed it before
        int to = from + nOfVoxelWeights;
        double[] voxelWeights = Arrays.copyOfRange(weights, from, to);
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
    Random random = new Random();
    Locomotion locomotion = new Locomotion(
        40,
        Locomotion.createTerrain("flat"),
        Lists.newArrayList(Locomotion.Metric.TRAVEL_X_VELOCITY),
        new Settings()
    );
    List<Sensor> sensors = List.of(  // list of sensors to use
        new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
        new Normalization(new AreaRatio())
    );
    int[] innerNeurons = new int[0]; // if more than 0 gives error: not enough heap memory

    GaussianPositionMapper mapper = new GaussianPositionMapper(true, 5, 10, 10, sensors,true, innerNeurons, 1);
    //System.out.println("lunghezza genotipo: " + mapper.getGenotypeSize()); // to know genotype size

    GaussianFactory<Double> factory = new GaussianFactory<>(mapper.getGenotypeSize(), 5);
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