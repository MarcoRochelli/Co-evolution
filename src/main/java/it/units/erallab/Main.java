

package it.units.erallab;

import com.google.common.base.Stopwatch;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.*;
import it.units.erallab.hmsrobots.tasks.locomotion.Footprint;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Point2;
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
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;
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
import java.util.stream.IntStream;

import static it.units.malelab.jgea.core.util.Args.*;

// mia versione
public class Main extends Worker {

  public static final int CACHE_SIZE = 10000;

  public Main(String[] args) {
    super(args);
  }

  public static void main(String[] args) {
    new Main(args);
  }

  @Override
  public void run() {
    Random random = new Random();
    // settings for the simulation
    final int numberOfValidations = 10;
    final int sizeOfvalidation = 3;
    final int nOfGaussians = 5;
    int[] innerNeurons = new int[0]; // array that sets number of inner neuron for each layer

    double episodeTime = d(a("episodeT", "2.0"));  // length of simulation
    int nBirths = i(a("nBirths", "10"));         // total number of births not robots
    int[] seeds = ri(a("seed", "0:1"));            // number of runs

    // THINGS I ADDED
    List<String> sizes = l(a("sizes", "5x5")); //5x5 or 10x10
    List<String> controllers = l(a("controllers", "homogeneous"));  // homogenous,heterogeneous or position
    List<String> sensorsConfig = l(a("sensorsConfig", "vel-area")); // vel-area or vel-area-touch
    List<String> representations = l(a("representation", "bit"));   // bit or gaussian
    List<String> signals = l(a("signal", "1"));   // can be 0,1,2 or 4

    List<String> terrainNames = l(a("terrain", "flat"));
    Function<Outcome, Double> fitnessFunction = Outcome::getCorrectedEfficiency;  // changed was relative before
    Function<Outcome, List<Item>> outcomeTransformer = o -> DataCollector.fromBean(
        o,
        true,
        Map.of(
            Double.TYPE, "%5.3f",
            List.class, "%30.30s",
            Grid.class, "%30.30s"
        ),
        Map.of(
            List.class, l -> ((List<?>) l).stream().map(Object::toString).collect(Collectors.joining(";")),
            Grid.class, g -> Grid.create((Grid<Boolean>) g, b -> b ? 'o' : '.').rows().stream()
                .map(r -> r.stream()
                    .map(c -> Character.toString(c))
                    .collect(Collectors.joining()))
                .collect(Collectors.joining(";"))
        )
    );
    List<String> validationOutcomeHeaders = outcomeTransformer.apply(prototypeOutcome()).stream().map(Item::getName).collect(Collectors.toList());
    List<String> validationTransformationNames = l(a("validationTransformations", "")).stream().filter(s -> !s.isEmpty()).collect(Collectors.toList());
    List<String> validationTerrainNames = l(a("validationTerrains", "flat")).stream().filter(s -> !s.isEmpty()).collect(Collectors.toList());
    if (!validationTerrainNames.isEmpty() && validationTransformationNames.isEmpty()) {
      validationTransformationNames.add("identity");
    }
    if (validationTerrainNames.isEmpty() && !validationTransformationNames.isEmpty()) {
      validationTerrainNames.add(terrainNames.get(0));
    }
    Settings physicsSettings = new Settings();
    //prepare file listeners
    MultiFileListenerFactory<Object, Robot<?>, Double> statsListenerFactory = new MultiFileListenerFactory<>((
        a("dir", "C:\\Users\\marco\\Desktop")),
        a("fileStats", "stats.txt")
    );

    MultiFileListenerFactory<Object, Robot<?>, Double> serializedListenerFactory = new MultiFileListenerFactory<>((
        a("dir", "C:\\Users\\marco\\Desktop")),
        a("fileSerialized", "serialized.txt")
    );

    CSVPrinter validationPrinter;
    List<String> validationKeyHeaders = List.of("seed", "terrain", "body", "mapper", "transformation", "evolver");
    try {
      if (a("validationFile", "validation.txt") != null) {
        validationPrinter = new CSVPrinter(new FileWriter(
            a("dir", "C:\\Users\\marco\\Desktop") + File.separator + a("validationFile", "validation.txt")
        ), CSVFormat.DEFAULT.withDelimiter(';'));
      } else {
        validationPrinter = new CSVPrinter(System.out, CSVFormat.DEFAULT.withDelimiter(';'));
      }
      List<String> headers = new ArrayList<>();
      headers.addAll(validationKeyHeaders);
      headers.addAll(List.of("validation.transformation", "validation.terrain"));
      headers.addAll(validationOutcomeHeaders.stream().map(n -> "validation." + n).collect(Collectors.toList()));
      validationPrinter.printRecord(headers);
    } catch (IOException e) {
      L.severe(String.format("Cannot create printer for validation results due to %s", e));
      return;
    }
    //shows params on log
    // L.info("number of processors "+Runtime.getRuntime().availableProcessors()); // gives the number of processors to put in ssh
    L.info("Terrains: " + terrainNames);
    L.info("Controller: " + controllers);
    L.info("Size: " + sizes);
    L.info("SensorConfig: " + sensorsConfig);
    L.info("Representation: " + representations);
    L.info("Signals: " + signals);
    //start iterations
    for (int seed : seeds) {
      for (String representation : representations) {
        for (String terrainName : terrainNames) {
          for (String controller : controllers) {
            for (String size : sizes) {
              for (String sensorConfig : sensorsConfig) {
                for (String signal : signals) {
                  Map<String, String> keys = new TreeMap<>(Map.of(
                      "seed", Integer.toString(seed),
                      "terrain", terrainName,
                      "controller", controller,
                      "size", size,
                      "sensor.config", sensorConfig,
                      "representation", representation,
                      "signals", signal
                  ));

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
                  int nOfSignals = switch (signal) {
                    case "0" -> 0;
                    case "1" -> 1;
                    case "2" -> 2;
                    case "4" -> 4;
                    default -> throw new IllegalArgumentException("incorrect signal string");
                  };

                  // wow i used inheritance to create mapper and factory!
                  Function<List<Double>, Robot<?>> mapper;
                  IndependentFactory<List<Double>> factory;

                  boolean control ;
                  switch (controller) {
                    case "homogeneous" -> control = false;
                    case "heterogeneous" -> control = true;
                    case "position" -> {
                      control = false;
                      mapper = new DoublePositionMapper(control, width, height, sensors, true, innerNeurons, nOfSignals);
                      UniformDoubleFactory udf = new UniformDoubleFactory(-1, 1);
                      factory = new FixedLengthListFactory<>(((DoublePositionMapper) mapper).getGenotypeSize(), udf);
                    }
                    case "positionGaussian" -> {
                      control = false;
                      mapper = new GaussianPositionMapper(control, nOfGaussians, width, height, sensors, true, innerNeurons, nOfSignals);
                      factory = new GaussianFactory<>(((GaussianPositionMapper) mapper).getGenotypeSize(), nOfGaussians);
                    }
                    default -> throw new IllegalArgumentException("incorrect controller string");
                  }

                  switch (representation) {
                    case "bit" -> {
                      mapper = new DoubleMapper(control, width, height, sensors, innerNeurons, nOfSignals);
                      UniformDoubleFactory udf = new UniformDoubleFactory(-1, 1);
                      factory = new FixedLengthListFactory<>(((DoubleMapper) mapper).getGenotypeSize(), udf);
                    }
                    case "gaussian" -> {
                      mapper = new GaussianMapper(control, nOfGaussians, width, height, sensors, innerNeurons, nOfSignals);
                      factory = new GaussianFactory<>(((GaussianMapper) mapper).getGenotypeSize(), nOfGaussians);
                    }
                    default -> throw new IllegalArgumentException("incorrect representation string");
                  }


                  //build training task
                  Function<Robot<?>, Outcome> trainingTask = Misc.cached(
                      new Locomotion(
                          episodeTime,
                          Locomotion.createTerrain(terrainName),
                          physicsSettings
                      ), CACHE_SIZE);

                  //build main data collectors for listener
                  List<DataCollector<?, ? super Robot<SensingVoxel>, ? super Double>> collectors = new ArrayList<DataCollector<?, ? super Robot<SensingVoxel>, ? super Double>>(List.of(
                      new Static(keys),
                      new Basic(),
                      new Population(),
                      new Diversity(),
                      new BestInfo("%5.2f"),
                      new FunctionOfOneBest<>(
                          ((Function<Individual<?, ? extends Robot<SensingVoxel>, ? extends Double>, Robot<SensingVoxel>>) Individual::getSolution)
                              .andThen(org.apache.commons.lang3.SerializationUtils::clone)
                              .andThen(trainingTask)
                              .andThen(outcomeTransformer)
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
                            new Item("serialized.robot", SerializationUtils.safelySerialize(i.getSolution()), "%s"),
                            new Item("serialized.genotype", SerializationUtils.safelySerialize((Serializable) i.getGenotype()), "%s")
                        ))
                    ).then(listener);
                  }
                  try {
                    Stopwatch stopwatch = Stopwatch.createStarted();
                    L.info(String.format("Starting %s", keys));

                    // CREATES THE EVOLVER
                    Evolver<?, Robot<?>, Double> evolver = new CMAESEvolver<>(
                        mapper,
                        factory,
                        PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness)
                        );
                    //optimize
                    Collection<Robot<?>> solutions = evolver.solve(
                        trainingTask.andThen(fitnessFunction),
                        new Births(nBirths),
                        random,
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
          }
        }
      }
    }
    try {
      validationPrinter.close(true);
    } catch (IOException e) {
      L.severe(String.format("Cannot close printer for validation results due to %s", e));
    }
  }

  private static Outcome prototypeOutcome() {
    return new Outcome(
        0d, 10d, 0d, 0d, 0d,
        new TreeMap<>(Map.of(0d, Point2.build(0d, 0d))),
        new TreeMap<>(IntStream.range(0, 100).boxed().collect(Collectors.toMap(
            i -> (double) i / 10d,
            i -> new Footprint(new boolean[]{true, false, true}))
        )),
        new TreeMap<>(Map.of(0d, Grid.create(1, 1, true)))
    );
  }
}
