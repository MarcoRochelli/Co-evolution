package it.units.erallab;

import com.google.common.base.Stopwatch;
import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.CentralizedSensing;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.PhaseSin;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.*;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.*;
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.MultiFileListenerFactory;
import it.units.malelab.jgea.core.listener.collector.*;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.selector.Worst;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.core.util.Pair;
import it.units.malelab.jgea.distance.Jaccard;
import it.units.malelab.jgea.representation.graph.*;
import it.units.malelab.jgea.representation.graph.numeric.Output;
import it.units.malelab.jgea.representation.graph.numeric.functiongraph.BaseFunction;
import it.units.malelab.jgea.representation.graph.numeric.functiongraph.FunctionGraph;
import it.units.malelab.jgea.representation.graph.numeric.functiongraph.FunctionNode;
import it.units.malelab.jgea.representation.graph.numeric.functiongraph.ShallowSparseFactory;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVPrinter;
import org.apache.commons.lang3.SerializationUtils;
import org.dyn4j.dynamics.Settings;

import java.io.*;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.Collectors;

import static it.units.malelab.jgea.core.util.Args.*;


public class MainCoEvo   {   // per il controllore a rete neurale distribuita extends Worker
/*
    public MainCoEvo(String[] args) throws FileNotFoundException {
        super(args);
    }

    public static void main(String[] args) throws FileNotFoundException {
        new MainCoEvo(args);
    }


    @Override
    public void run() {
        // altezza e larghezza massime del robot
        int width = 5;
        int height = 5;

        //read parameters TOLTO SHAPE NAMES
        int[] runs = ri(a("run", "1"));
        List<String> terrainNames = l(a("terrain", "flat")); //flat,uneven5
        List<String> evolverNames = l(a("evolver", "standard-2op")); //mutationOnly,standardDiv-1|2op,standard-1|2op  regole sul come si evolve
        List<String> controllerNames = l(a("controller", "phases"));  // tipi di controllore
        double finalT = d(a("finalT", "60"));    // quanto dura la simulazione
        double minDT = d(a("minDT", "0.0333"));   // ha a che fare col tempo di campionamento del segnale generato dal controllore
        double maxDT = d(a("maxDT", "0.0333"));
        List<Double> drivingFrequencies = d(l(a("drivingF", "-1")));
        List<Double> mutationSigmas = d(l(a("mutationSigma", "0.15")));
        List<Integer> controlStepIntervals = i(l(a("controlStepInterval", "1")));  // va usato in sinergia con minDT
        int nPop = i(a("npop", "5"));                                 // quanti robottini simulo
        int iterations = i(a("iterations", "10"));                    // quante generazioni faccio
        int cacheSize = i(a("cacheSize", "1000"));
        boolean statsToStandardOutput = b(a("stout", "false"));
        List<Locomotion.Metric> metrics = Lists.newArrayList(
                Locomotion.Metric.TRAVEL_X_RELATIVE_VELOCITY // QUA SI CAMBIA LA FITNESS DA RELATIVA AD ALTRO IN CASO
        );
        //set voxel builder
        Voxel.Description builder = Voxel.Description.build();
        //prepare things                       // mettere bene i percorsi potrebbe essere da cambiare i nomi dei file
        MultiFileListenerFactory statsListenerFactory = new MultiFileListenerFactory(a("dir", "C:\\Users\\marco\\Desktop\\Tesi\\HMSRevo"), a("fileStats", "stats.txt"));
        MultiFileListenerFactory serializedBestListenerFactory = new MultiFileListenerFactory(a("dir", "C:\\Users\\marco\\Desktop\\Tesi\\HMSRevo"), a("fileSerialized", "serialized.txt"));
        //write config
        if (a("fileConfig", null) != null) {
            try {
                Map<String, Object> builderProperties = PropertyUtils.describe(builder);
                try (PrintStream filePS = new PrintStream(a("dir", "") + File.separator + a("fileConfig", null))) {
                    for (Map.Entry<String, Object> entry : builderProperties.entrySet()) {
                        filePS.printf("%s : %s%n", entry.getKey(), entry.getValue().toString());
                    }
                }
            } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException ex) {
                L.severe( String.format("Cannot read builder properties due to %s", ex));
            } catch (FileNotFoundException ex) {
                L.severe(String.format("Cannot wrtie builder properties on file due to %s", ex));
            }
        }
        //iterate
        for (int run : runs) {
            for (String terrainName : terrainNames) {
                for (String evolverName : evolverNames) {
                    for (String controllerName : controllerNames) {
                        for (int controlStepInterval : controlStepIntervals) {
                            for (double mutationSigma : mutationSigmas) {
                                for (double drivingFrequency : drivingFrequencies) {
                                    //il problema da risolvere NON SI TOCCA
                                    LocomotionProblem problem = new LocomotionProblem(
                                            finalT, minDT, maxDT,
                                            Locomotion.createTerrain(terrainName),
                                            controlStepInterval,
                                            metrics,
                                            LocomotionProblem.ApproximationMethod.FINAL_T
                                    );

                                    // PARTE CHE HO SCRITTO IO
                                    Grid<Boolean> body = null;
                                    Grid<double[]> controller = null;
                                    Pair<Grid<Boolean>, Grid<double[]>> pair;
                                    pair = Pair.build(body, controller);





                                    //PRIMA DELLA FACTORY BISOGNA CAPIRE QUANTO è LUNGA LA RETE NEURALE FACENDO UN ROBOTTINO DI PROVA
                                    final Grid<Boolean> structure = Grid.create(5, 5, true);
                                    // bulds a grid with sensors
                                    Grid<Voxel.Description> distRobotWithSensors = Grid.create(structure.getW(), structure.getH(), (x, y) -> {
                                        if (structure.get(x, y)) {
                                            Voxel.Description d = Voxel.Description.build();
                                            d.getSensors().add(new Average(new Velocity(true, 2d * d.getSideLength(), Velocity.Axis.X, Velocity.Axis.Y), 0.1d));
                                            d.getSensors().add(new Average(new AreaRatio(), 0.1d));
                                            return d;
                                        }
                                        return null;
                                    });
                                    // builds robot example
                                    Robot.Description distributedMlpRobot = new Robot.Description(
                                            distRobotWithSensors,
                                            new DistributedMLP(distRobotWithSensors, new int[0], 1)
                                    );
                                    Random ran = new Random();
                                    double[] weights = ((DistributedMLP) distributedMlpRobot.getController()).getParams();
                                    for (int i = 0; i < weights.length; i++) {
                                        weights[i] = ran.nextDouble() * 2d - 1d;
                                    }
                                    ((DistributedMLP) distributedMlpRobot.getController()).setParams(weights);

                                    //MANCA DA PASSARE LA LUNGHEZZA DELLA RETE NEURALE ALLA FACTORY O AL MAPPER


                                    //weights.length is length of the neural network??
                                    Factory<Pair<Grid<Boolean>, Grid<double[]>>> factory = null;
                                    NonDeterministicFunction<Pair<Grid<Boolean>, Grid<double[]>>, Robot.Description> mapper = null;

                                    factory = new PairSequenceFactory(-Math.PI, Math.PI, width, height);
                                    //manca da capire sta roba sul sensor grid !!!!!!!!!!
                                    mapper = Mapper.getSensingMapper(builder, weights.length, run, drivingFrequency, runs);






                                    //prepare evolver  PENSO SIA OK
                                    Evolver<Pair<Grid<Boolean>, Grid<double[]>>, Robot.Description, List<Double>> evolver = null;

                                    if (evolverName.equals("mutationOnly")) {
                                        // prima era così macome è sotto va bene cmq se non meglio evolver2 = new MutationOnly<Pair<Grid<Boolean>, Grid<Double>>, VoxelCompound.Description, List<Double>>(
                                        evolver = new MutationOnly<>(
                                                nPop,
                                                factory,
                                                new ParetoRanker<>(false),
                                                mapper,
                                                new PairGaussianMutation(mutationSigma),
                                                Lists.newArrayList(new Iterations(iterations)),
                                                cacheSize,
                                                false
                                        );
                                    } else if (evolverName.startsWith("standard")) {
                                        Crossover<Pair<Grid<Boolean>, Grid<double[]>>> crossover;
                                        crossover = new PairUniformCrossover(Range.closedOpen(-1d, 2d));

                                        Mutation<Pair<Grid<Boolean>, Grid<double[]>>> mutation;
                                        mutation = new PairGaussianMutation(mutationSigma);

                                        Map<GeneticOperator<Pair<Grid<Boolean>, Grid<double[]>>>, Double> operators = new LinkedHashMap<>();

                                        if (evolverName.split("-")[1].equals("1op")) {
                                            operators.put(crossover.andThen(mutation), 1d);
                                        } else if (evolverName.split("-")[1].equals("2op")) {
                                            operators.put(crossover, 0.8d);
                                            operators.put(mutation, 0.2d);
                                        }
                                        if (evolverName.startsWith("standardDiv")) {
                                            evolver = new StandardWithEnforcedDiversity<Pair<Grid<Boolean>, Grid<double[]>>, Robot.Description, List<Double>>(
                                                    100,
                                                    nPop,
                                                    factory,
                                                    new ParetoRanker<>(false),
                                                    mapper,
                                                    operators,
                                                    new Tournament<>(Math.max(Math.round(nPop / 30), 2)),
                                                    new Worst(),
                                                    nPop,
                                                    true,
                                                    Lists.newArrayList(new Iterations(iterations)),
                                                    cacheSize
                                            );
                                        } else {
                                            evolver = new StandardEvolver<Pair<Grid<Boolean>, Grid<double[]>>, Robot.Description, List<Double>>(
                                                    nPop,
                                                    factory,
                                                    new ParetoRanker<>(false),
                                                    mapper,
                                                    operators,
                                                    new Tournament<>(Math.max(Math.round(nPop / 30), 2)),
                                                    new Worst(),
                                                    nPop,
                                                    true,
                                                    Lists.newArrayList(new Iterations(iterations)),
                                                    cacheSize,
                                                    false
                                            );
                                        }
                                    }
                                    //prepare keys   NON SI TOCCA ALMENO PENSO
                                    Map<String, String> keys = new LinkedHashMap<>();
                                    keys.put("evolver", evolverName);
                                    keys.put("control.step.interval", Integer.toString(controlStepInterval));
                                    keys.put("controller", controllerName);
                                    keys.put("run", Integer.toString(run));
                                    keys.put("n.pop", Integer.toString(nPop));
                                    keys.put("driving.frequency", Double.toString(drivingFrequency));
                                    keys.put("mutation.sigma", Double.toString(mutationSigma));
                                    keys.put("shape", "5x5");                                 // QUI CE SHAPE
                                    keys.put("terrain", terrainName);
                                    keys.put("metrics", metrics.stream().map((m) -> m.toString().toLowerCase().replace("_", ".")).collect(Collectors.joining("/")));
                                    L.info(String.format("Keys: %s", keys));
                                    //prepare collectors   NON SI TOCCA
                                    List<DataCollector> statsCollectors = Lists.newArrayList(
                                            new Static(keys),
                                            new Basic(),
                                            new Population(),
                                            new Diversity(),
                                            new BestInfo<>(problem.getFitnessFunction(metrics), "%+5.3f"),
                                            new FunctionOfBest<>(
                                                    "valid",
                                                    problem.getFitnessFunction(Lists.newArrayList(Locomotion.Metric.values())),
                                                    Arrays.stream(Locomotion.Metric.values()).map((m) -> {
                                                        return m.toString().toLowerCase().replace('_', '.');
                                                    }).collect(Collectors.toList()),
                                                    Collections.singletonList("%+5.3f")
                                            ),// questa sotto è la righa che ho aggiunto per rendere confrontabili lo statse il serialized
                                            new FunctionOfBest("serialized", (Individual individual) -> Collections.singletonList(new Item("description", Util.lazilySerialize((Serializable) individual.getSolution()), "%s")))
                                    );
                                    List<DataCollector> serializedCollectors = Lists.newArrayList(
                                            new Static(keys),
                                            new Basic(),
                                            new BestInfo<>(problem.getFitnessFunction(metrics), "%+5.3f"),
                                            new FunctionOfBest("serialized", (Individual individual) -> Collections.singletonList(new Item("description", Util.lazilySerialize((Serializable) individual.getSolution()), "%s")))
                                    );
                                    //run evolver    NON SI TOCCA
                                    Random r = new Random(run);
                                    Listener listener = statsListenerFactory.build(
                                            statsCollectors.toArray(new DataCollector[statsCollectors.size()])
                                    ).then(serializedBestListenerFactory.build(
                                            serializedCollectors.toArray(new DataCollector[serializedCollectors.size()])
                                    ));
                                    if (statsToStandardOutput) {
                                        listener = listener.then(listener(statsCollectors.toArray(new DataCollector[statsCollectors.size()])));
                                    }
                                    try {
                                        evolver.solve(problem, r, executorService, Listener.onExecutor(listener, executorService));
                                    } catch (InterruptedException | ExecutionException ex) {

                                        L.severe (String.format("Cannot solve problem: %s", ex));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
*/
}