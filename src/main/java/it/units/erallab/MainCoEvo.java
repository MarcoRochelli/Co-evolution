package it.units.erallab;

import com.google.common.base.Stopwatch;
import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.AreaRatio;
import it.units.erallab.hmsrobots.core.sensors.Normalization;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.core.sensors.Velocity;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.CMAESEvolver;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.MultiFileListenerFactory;
import it.units.malelab.jgea.core.listener.collector.*;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.selector.Worst;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;
import org.apache.commons.lang3.SerializationUtils;
import org.dyn4j.dynamics.Settings;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Serializable;
import java.util.*;
import java.util.concurrent.ExecutionException;
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
  public void run() {  // working basic version
    Random random = new Random();
    // settings for the simulation
    double episodeTime = d(a("episodeT", "2.0"));  // length of simulation
    int nBirths = i(a("nBirths", "2000"));           // total number of births not robots
    int[] seeds = ri(a("seed", "0:1"));             // number of runs
    // sensors the voxel should have
    List<Sensor> sensors = List.of(  // list of sensors to use
        new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
        new Normalization(new AreaRatio())
    );
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
       /* a("dir", ".")),  // where to save should i change this? i didn't have to in old code
        a("fileStats", null)              // how to name it

        */
        a("dir", "C:\\Users\\marco\\Desktop")),
        a("fileStats", "stats.txt")
    );
    MultiFileListenerFactory<Object, Robot<?>, Double> serializedListenerFactory = new MultiFileListenerFactory<>((
        /*a("dir", ".")),
        a("fileSerialized",null)

         */
        a("dir", "C:\\Users\\marco\\Desktop")),
        a("fileSerialized", "serialized.txt")
    );

    //shows params on log
    L.info("Terrains: " + terrainNames);

    //start iterations
    for (int seed : seeds) {
      for (String terrainName : terrainNames) {
        Map<String, String> keys = new TreeMap<>(Map.of(
            "seed", Integer.toString(seed),
            "terrain", terrainName
        ));

        //problem to solve
        Function<Robot<?>, List<Double>> trainingTask = Misc.cached(
            new Locomotion(
                episodeTime,
                Locomotion.createTerrain(terrainName),
                allMetrics,
                physicsSettings
            ), CACHE_SIZE);


        // cretes mapper, factory, genotype and robot
        DistributedMapper mapper = new DistributedMapper(5, 5, sensors, innerNeurons, 1);
        UniformDoubleFactory udf = new UniformDoubleFactory(-1, 1);
        FixedLengthListFactory<Double> factory = new FixedLengthListFactory<>(mapper.getGenotypeSize(), udf);

        // robot should evolve
        try {
          Stopwatch stopwatch = Stopwatch.createStarted();
          L.info(String.format("Starting %s", keys));

          // CREATES THE EVOLVER
          Evolver<List<Double>, Robot<?>, Double> evolver = new CMAESEvolver<>(
              mapper,
              factory,
              PartialComparator.from(Double.class).comparing(Individual::getFitness),
              -1,
              1
          );

          //build main data collectors for listener
          List<DataCollector<?, ? super Robot<?>, ? super Double>> collectors = new ArrayList<DataCollector<?, ? super Robot<?>, ? super Double>>(List.of(
              new Static(keys),
              new Basic(),
              new Population(),
              new Diversity(),
              new BestInfo("%5.2f"),
              new FunctionOfOneBest<>( // i do not understand this
                  ((Function<Individual<?, ? extends Robot<?>, ? extends Double>, Robot<?>>) Individual::getSolution)
                      .andThen(SerializationUtils::clone)
                      .andThen(metrics(allMetrics, "training", trainingTask, "%6.2f"))
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
          Collection<Robot<?>> solutions = evolver.solve( // here uses evolver to solve the problem
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

        } catch (InterruptedException | ExecutionException e) {
          L.severe(String.format("Cannot complete %s due to %s",
              keys,
              e
          ));
          e.printStackTrace();
        }
      }

    }
  }

  private static Function<Robot<?>, List<Item>> metrics(List<Locomotion.Metric> metrics, String prefix, Function<Robot<?>, List<Double>> task, String format) {
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
