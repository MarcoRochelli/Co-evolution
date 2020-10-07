package it.units.erallab;

import com.google.common.base.Stopwatch;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.sensors.*;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.IndependentFactory;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.CMAESEvolver;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.MultiFileListenerFactory;
import it.units.malelab.jgea.core.listener.collector.*;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.lang3.SerializationUtils;
import org.dyn4j.dynamics.Settings;

import java.io.Serializable;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;
import java.util.stream.Collectors;

import static it.units.malelab.jgea.core.util.Args.*;

public class MainCoEvo extends Worker {

  public static final int CACHE_SIZE = 10000;

  public MainCoEvo(String[] args) {
    super(args);
  }

  public static void main(String[] args) {
    new MainCoEvo(args);
  }

  @Override
  public void run() {
    // settings for the simulation
    double episodeTime = d(a("episodeT", "2.0"));  // length of simulation
    int nBirths = i(a("nBirths", "50"));         // total number of births not robots
    int[] seeds = ri(a("seed", "0:1"));            // number of runs

    // THINGS I ADDED
    List<String> sizes = l(a("sizes", "10x10,5x5")); //5x5 or 10x10
    List<String> controllers = l(a("controllers", "heterogeneous,homogeneous"));  // homogenous or heterogeneous
    List<String> sensorsConfig = l(a("sensorsConfig", "vel-area-touch")); // vel-area or vel-area-touch
    List<String> representations = l(a("representation", "gaussian"));   // bit or gaussian or position


    // inner neurons
    int[] innerNeurons = new int[0]; // array that sets number of inner neuron foe each layer
    List<String> terrainNames = l(a("terrain", "flat"));   //flat
    Locomotion.Metric fitnessMetric = Locomotion.Metric.valueOf(a("fitnessMetric", Locomotion.Metric.TRAVEL_X_RELATIVE_VELOCITY.name().toLowerCase()).toUpperCase());
    List<Locomotion.Metric> allMetrics = l(a("metrics", List.of(Locomotion.Metric.values()).stream().map(m -> m.name().toLowerCase()).collect(Collectors.joining(",")))).stream()
        .map(String::toUpperCase)
        .map(Locomotion.Metric::valueOf)
        .collect(Collectors.toList());
    if (!allMetrics.contains(fitnessMetric)) {
      allMetrics.add(fitnessMetric);
    }

    Settings physicsSettings = new Settings();

    //prepare file listeners
    MultiFileListenerFactory<Object, Robot<?>, Double> statsListenerFactory = new MultiFileListenerFactory<>((
        /*
        a("dir", ".")),  // where to save should i change this? i didn't have to in old code
        a("fileStats", null)              // how to name it
         */

        // to create a file to check if it works
        a("dir", "C:\\Users\\marco\\Desktop")),
        a("fileStats", "stats.txt")

    );
    MultiFileListenerFactory<Object, Robot<?>, Double> serializedListenerFactory = new MultiFileListenerFactory<>((
        /*
        a("dir", ".")),
        a("fileSerialized", null)
         */

        // to create a file to check if it works
        a("dir", "C:\\Users\\marco\\Desktop")),
        a("fileSerialized", "serialized.txt")

    );
    //shows params on log
    L.info("Terrains: " + terrainNames);
    L.info("Controller: " + controllers);
    L.info("Size: " + sizes);
    L.info("SensorConfig: " + sensorsConfig);
    L.info("Representation: " + representations);

    //start iterations
    for (int seed : seeds) {
      for (String representation : representations) {
        for (String terrainName : terrainNames) {
          for (String controller : controllers) {
            for (String size : sizes) {
              for (String sensorConfig : sensorsConfig) {
                Map<String, String> keys = new TreeMap<>(Map.of(
                    "seed", Integer.toString(seed),
                    "terrain", terrainName,
                    "controller", controller,
                    "size", size,
                    "sensor.config", sensorConfig,
                    "representation", representation
                ));
                //problem to solve
                Function<Robot<?>, List<Double>> trainingTask = Misc.cached(
                    new Locomotion(
                        episodeTime,
                        Locomotion.createTerrain(terrainName),
                        allMetrics,
                        physicsSettings
                    ), CACHE_SIZE);

                int width;
                int height;
                if (size.equals("5x5")) {
                  width = 5;
                  height = 5;
                } else if (size.equals("10x10")) {
                  width = 10;
                  height = 10;
                } else {
                  throw new IllegalArgumentException("incorrect size string");
                }

                List<Sensor> sensors;  // list of sensors to use
                if (sensorConfig.equals("vel-area")) {
                  // sensors the voxel should have
                  sensors = List.of(
                      new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
                      new Normalization(new AreaRatio())
                  );
                } else if (sensorConfig.equals("vel-area-touch")) {
                  sensors = List.of(
                      new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
                      new Normalization(new AreaRatio()),
                      new Normalization(new Average(new Touch(), 0.5)) // Eric said that it is better to add average with 0.5
                  );
                } else {
                  throw new IllegalArgumentException("incorrect sensor string");
                }
                boolean control;
                if (controller.equals("homogeneous")) {
                  control = false;
                } else if (controller.equals("heterogeneous")) {
                  control = true;
                } else {
                  throw new IllegalArgumentException("incorrect controller string");
                }

                // wow i used ereditariety to create mapper and factory!
                Function<List<Double>, Robot<?>> mapper;
                IndependentFactory<List<Double>> factory;
                // creates mapper and factory GAUSSIAN REPRESENTATION
                switch (representation) {
                  case "bit" -> {
                    // creates mapper and factory BIT REPRESENTATION
                    mapper = new DoubleMapper(control, width, height, sensors, innerNeurons, 1);
                    UniformDoubleFactory udf = new UniformDoubleFactory(-1, 1);
                    factory = new FixedLengthListFactory<>(((DoubleMapper) mapper).getGenotypeSize(), udf);
                  }
                  case "gaussian" -> {
                    mapper = new GaussianMapper(control, 5, width, height, sensors, innerNeurons, 1);
                    factory = new GaussianFactory<>(((GaussianMapper) mapper).getGenotypeSize(), 5);
                  }
                  case "position" -> {
                    // creates mapper and factory POSITION REPRESENTATION
                    mapper = new DoublePositionMapper(control, width, height, sensors, true, innerNeurons, 1);
                    UniformDoubleFactory udf = new UniformDoubleFactory(-1, 1);
                    factory = new FixedLengthListFactory<>(((DoublePositionMapper) mapper).getGenotypeSize(), udf);
                  }
                  default -> throw new IllegalArgumentException("incorrect representation string");
                }

                // to evolve the robot
                try {
                  Stopwatch stopwatch = Stopwatch.createStarted();
                  L.info(String.format("Starting %s", keys));

                  // CREATES THE EVOLVER
                  Evolver<List<Double>, Robot<?>, Double> evolver = new CMAESEvolver<>(
                      mapper,
                      factory,
                      PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness), // reversed is ABSOLUTELY NECESSARY i was minimizing instead of maximizing
                      -1,
                      1
                  );

                  //build main data collectors for listener saves things on .txt
                  List<DataCollector<?, ? super Robot<?>, ? super Double>> collectors = new ArrayList<DataCollector<?, ? super Robot<?>, ? super Double>>(List.of(
                      new Static(keys),
                      new Basic(),
                      new Population(),
                      new Diversity(),
                      new BestInfo("%5.2f"),
                      // if deleted file stats does not have last 10 columns
                      new FunctionOfOneBest<>(
                          ((Function<Individual<?, ? extends Robot<?>, ? extends Double>, Robot<?>>) Individual::getSolution)
                              .andThen(SerializationUtils::clone)
                              .andThen(metrics(allMetrics, "training", trainingTask, "%6.2f"))
                      ),
                      // save number of effective voxel of the robot and effective height and effective width
                      new FunctionOfOneBest<>(
                          individual -> List.of(
                              new Item("robot.size", individual.getSolution().getVoxels().count(Objects::nonNull), "%2d"),
                              //new Item("robot.size", individual.getSolution().getVoxels().stream().filter(Objects::nonNull).count(), "%2d"), // gives always width*height
                              new Item("robot.width", it.units.erallab.hmsrobots.util.Utils.cropGrid(individual.getSolution().getVoxels(), Objects::nonNull).getW(), "%2d"),
                              new Item("robot.height", it.units.erallab.hmsrobots.util.Utils.cropGrid(individual.getSolution().getVoxels(), Objects::nonNull).getH(), "%2d")
                          )
                      )
                  ));
                  Listener<? super Object, ? super Robot<?>, ? super Double> listener;
                  if (statsListenerFactory.getBaseFileName() == null) {
                    listener = listener(collectors.toArray(DataCollector[]::new));
                  } else {
                    listener = statsListenerFactory.build(collectors.toArray(DataCollector[]::new));
                  }
                  if (serializedListenerFactory.getBaseFileName() != null) {
                    listener = serializedListenerFactory.build(
                        new Static(keys),
                        new Basic(),
                        new FunctionOfOneBest<>(i -> List.of(
                            new Item("fitness.value", i.getFitness(), "%7.5f"),
                            new Item("serialized.robot", Utils.safelySerialize(i.getSolution()), "%s"),
                            new Item("serialized.genotype", Utils.safelySerialize((Serializable) i.getGenotype()), "%s")
                        ))
                    ).then(listener);
                  }
                  Collection<Robot<?>> solutions = evolver.solve( // here uses the evolver to solve the problem
                      trainingTask.andThen(values -> values.get(allMetrics.indexOf(fitnessMetric))),
                      new Births(nBirths),
                      new Random(seed),
                      executorService,
                      Listener.onExecutor(
                          listener,
                          executorService
                      )
                  );
                  L.info(String.format("Done %s: %d solutions in %4ds",
                      keys,
                      solutions.size(),
                      stopwatch.elapsed(TimeUnit.SECONDS)
                  ));

                } catch (Throwable e) {
                  L.severe(String.format("Cannot complete %s due to %s",
                      keys,
                      e
                  ));
                  e.printStackTrace();
                }
              }
            }
          }
        }
      }
    }
  }

  private static Function<Robot<?>, List<Item>> metrics(List<Locomotion.Metric> metrics, String
      prefix, Function<Robot<?>, List<Double>> task, String format) {
    return individual -> {
      List<Double> values = task.apply(individual);
      List<Item> items = new ArrayList<>(metrics.size());
      for (int i = 0; i < metrics.size(); i++) {
        items.add(new Item(
            prefix + "." + metrics.get(i).name().toLowerCase(),
            values.get(i),
            format
        ));
      }
      return items;
    };
  }
}
