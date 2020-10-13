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
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
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

import static java.lang.Math.*;

public class GaussianMapper implements Function<List<Double>, Robot<?>> {

  private final boolean heterogeneous;
  private final int nOfGaussians;
  private final int width;
  private final int height;

  private final List<Sensor> sensors;
  private final int[] innerNeurons;
  private final int signals;
  private final static double THRESHOLD = 0d;

  public GaussianMapper(boolean heterogeneous, int nOfGaussians, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
    this.heterogeneous = heterogeneous;
    this.nOfGaussians = nOfGaussians;
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
    int nOfBodyParams = 5 * nOfGaussians;
    if (heterogeneous) {
      return nOfBodyParams + nOfWeights * width * height;
    } else
      return nOfBodyParams + nOfWeights;
  }

  @Override
  public Robot<?> apply(List<Double> genotype) {
    int nOfBodyParams = 5 * nOfGaussians;
    int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum();
    int nOfOutputs = signals * 4 + 1;
    int nOfVoxelWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));
    if (genotype.size() != getGenotypeSize()) {
      throw new IllegalArgumentException("Sensor list has wrong dimension");
    }

    SensingVoxel sensingVoxel = new SensingVoxel(sensors);

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

        //value += weight * (exp(-0.5 * (pow((x - Ux) / Ox, 2.0) + pow((y - Uy) / Oy, 2.0))) / (2 * PI * Ox * Oy)); // same as above i just simplified exponentials

        /*
        System.out.println("coordinate: " + entry.getX() + ","+ entry.getY());
        System.out.println("i: " + i);
        System.out.println("weight: " + weight);
        System.out.println("Ux: " + Ux);
        System.out.println("Uy: " + Uy);
        System.out.println("Ox: " + Ox);
        System.out.println("Oy: " + Oy);
        System.out.println("value: " + value);
         */

      }
      values.set(entry.getX(), entry.getY(), value);
    }


    Grid<SensingVoxel> body = Grid.create(width, height);
    // for each value of values if it is bigger tha a threshold puts a voxel in that position
    for (Grid.Entry<Double> entry : values) {
      if (entry.getValue() > THRESHOLD) {
        body.set(entry.getX(), entry.getY(), SerializationUtils.clone(sensingVoxel));
      } else {
        body.set(entry.getX(), entry.getY(), null);
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
        int from = entry.getX() * nOfVoxelWeights + entry.getY() * width * nOfVoxelWeights; //  + width * height is not needed because i remove it before
        int to = from + nOfVoxelWeights;
        double[] voxelWeights = Arrays.copyOfRange(weights, from, to);
        /*
        System.out.println("parto da: " + from);
        System.out.println("fino a: " + to);
         */
        mlp.setParams(voxelWeights);
      } else {
        mlp.setParams(weights);
      }
      distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp);
    }

    return new Robot<>(
        distributedSensing,
        body
    );
  }


  // to test the mapper
  public static void main(String[] args) {
    Random random = new Random();

    // problem to solve
    Locomotion locomotion = new Locomotion(
        40,
        Locomotion.createTerrain("flat"),
        new Settings()
    );

    List<Sensor> sensors = List.of(  // list of sensors to use
        new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
        new Normalization(new AreaRatio())
    );
    int[] innerNeurons = new int[0]; // if more than 0 gives error: not enough heap memory

    GaussianMapper mapper = new GaussianMapper(true, 5, 10, 10, sensors, innerNeurons, 1);
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
