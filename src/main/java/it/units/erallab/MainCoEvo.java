package it.units.erallab;

import com.google.common.base.Stopwatch;
import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.CentralizedSensing;
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
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
import java.util.stream.IntStream;

import static it.units.malelab.jgea.core.util.Args.*;

// mia versione
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
        // settings for the simulation UNDERSTAND THEM ALL
        double episodeTime = d(a("episodeT", "10.0"));  // length of simulation?
        int nBirths = i(a("nBirths", "500"));           // total number of robots simulated
        //HOW DO I CONTROL ROBOT FOR EVERY GENERATION????????
        int[] seeds = ri(a("seed", "0:1"));             // ???? WHAT IS THIS
        //int validationBirthsInterval = i(a("validationBirthsInterval", "100"));
        List<String> terrainNames = l(a("terrain", "flat"));   //flat,uneven5 or what??
        List<String> evolverMapperNames = l(a("evolver", "mlp-0.65-cmaes")); // rules of how to evolve (ex mutationOnly,standardDiv-1|2op,standard-1|2op)
        List<String> bodyNames = l(a("body", "biped-4x3-f-f"));  // body i thnk i do not need this HOW TO MAKE A 10X10 BODY???
        List<String> transformationNames = l(a("transformations", "identity"));
        List<String> robotMapperNames = l(a("mapper", "centralized"));  // mapper tipe i MUST PUT DISTRIBUTED
        Locomotion.Metric fitnessMetric = Locomotion.Metric.valueOf(a("fitnessMetric", Locomotion.Metric.X_DISTANCE_CORRECTED_EFFICIENCY.name().toLowerCase()).toUpperCase());
        List<Locomotion.Metric> allMetrics = l(a("metrics", List.of(Locomotion.Metric.values()).stream().map(m -> m.name().toLowerCase()).collect(Collectors.joining(",")))).stream()
                .map(String::toUpperCase)
                .map(Locomotion.Metric::valueOf)
                .collect(Collectors.toList());
        if (!allMetrics.contains(fitnessMetric)) {
            allMetrics.add(fitnessMetric);
        }
        List<String> validationTransformationNames = l(a("validationTransformations", "")).stream().filter(s -> !s.isEmpty()).collect(Collectors.toList());
        List<String> validationTerrainNames = l(a("validationTerrains", "")).stream().filter(s -> !s.isEmpty()).collect(Collectors.toList());
        if (!validationTerrainNames.isEmpty() && validationTransformationNames.isEmpty()) {
            validationTransformationNames.add("identity");
        }
        if (validationTerrainNames.isEmpty() && !validationTransformationNames.isEmpty()) {
            validationTerrainNames.add(terrainNames.get(0));
        }
        Settings physicsSettings = new Settings();

        //prepare file listeners
        MultiFileListenerFactory<Object, Robot<?>, Double> statsListenerFactory = new MultiFileListenerFactory<>((
                a("dir", "C:\\Users\\marco\\Desktop")),  // where to save should i change this? i didn't have to in old code
                a("fileStats", "stats.txt")              // how to name it
        );
        MultiFileListenerFactory<Object, Robot<?>, Double> serializedListenerFactory = new MultiFileListenerFactory<>((
                a("dir", "C:\\Users\\marco\\Desktop")),
                a("fileSerialized", "serialized.txt")
        );

        // things to save the result
        CSVPrinter validationPrinter;
        List<String> validationKeyHeaders = List.of("seed", "terrain", "body", "mapper", "transformation", "evolver");
        try {
            if (a("validationFile", null) != null) {
                validationPrinter = new CSVPrinter(new FileWriter(
                        a("dir", ".") + File.separator + a("validationFile", null)
                ), CSVFormat.DEFAULT.withDelimiter(';'));
            } else {
                validationPrinter = new CSVPrinter(System.out, CSVFormat.DEFAULT.withDelimiter(';'));
            }
            List<String> headers = new ArrayList<>();
            headers.addAll(validationKeyHeaders);
            headers.addAll(List.of("validation.transformation", "validation.terrain"));
            headers.addAll(allMetrics.stream().map(m -> m.toString().toLowerCase()).collect(Collectors.toList()));
            validationPrinter.printRecord(headers);
        } catch (IOException e) {
            L.severe(String.format("Cannot create printer for validation results due to %s", e));
            return;
        }

        //shows params on log
        L.info("Evolvers: " + evolverMapperNames);
        L.info("Mappers: " + robotMapperNames);
        L.info("Bodies: " + bodyNames);
        L.info("Terrains: " + terrainNames);
        L.info("Transformations: " + transformationNames);
        L.info("Validations: " + Lists.cartesianProduct(validationTerrainNames, validationTransformationNames));

        //start iterations
        for (int seed : seeds) {
            for (String terrainName : terrainNames) {
                for (String bodyName : bodyNames) {
                    for (String robotMapperName : robotMapperNames) {
                        for (String transformationName : transformationNames) {
                            for (String evolverMapperName : evolverMapperNames) {
                                Map<String, String> keys = new TreeMap<>(Map.of(
                                        "seed", Integer.toString(seed),
                                        "terrain", terrainName,
                                        "body", bodyName,
                                        "mapper", robotMapperName,
                                        "transformation", transformationName,
                                        "evolver", evolverMapperName
                                ));
                                //build training task
                                Function<Robot<?>, List<Double>> trainingTask = Misc.cached(
                                        Utils.buildRobotTransformation(transformationName).andThen(new Locomotion(
                                                episodeTime,
                                                Locomotion.createTerrain(terrainName),
                                                allMetrics,
                                                physicsSettings
                                        )), CACHE_SIZE);
                                //build main data collectors for listener
                                List<DataCollector<?, ? super Robot<SensingVoxel>, ? super Double>> collectors = new ArrayList<DataCollector<?, ? super Robot<SensingVoxel>, ? super Double>>(List.of(
                                        new Static(keys),
                                        new Basic(),
                                        new Population(),
                                        new Diversity(),
                                        new BestInfo("%5.2f"),
                                        new FunctionOfOneBest<>(
                                                ((Function<Individual<?, ? extends Robot<SensingVoxel>, ? extends Double>, Robot<SensingVoxel>>) Individual::getSolution)
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


                                // CREATES THE BODY
                                Grid<? extends SensingVoxel> body = buildBody(bodyName);

                                try {
                                    Stopwatch stopwatch = Stopwatch.createStarted();
                                    L.info(String.format("Starting %s", keys));
                                    // CREATES THE EVOLVER i think i do not have tho change this
                                    Evolver<?, Robot<?>, Double> evolver = buildEvolverMapper(evolverMapperName).apply(buildRobotMapper(robotMapperName), body);
                                    //optimize ? Does the evolution? i think i do not have to change this
                                    Collection<Robot<?>> solutions = evolver.solve(
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
                                    //do validation   WHAT IS VALIDATION ? was it present in old code?
                                    for (String validationTransformationName : validationTransformationNames) {
                                        for (String validationTerrainName : validationTerrainNames) {
                                            //build validation task
                                            Function<Robot<?>, List<Double>> validationTask = new Locomotion(
                                                    episodeTime,
                                                    Locomotion.createTerrain(validationTerrainName),
                                                    allMetrics,
                                                    physicsSettings
                                            );
                                            validationTask = Utils.buildRobotTransformation(validationTransformationName)
                                                    .andThen(SerializationUtils::clone)
                                                    .andThen(validationTask);
                                            List<Double> metrics = validationTask.apply(solutions.stream().findFirst().get());
                                            L.info(String.format(
                                                    "Validation %s/%s of \"first\" best done",
                                                    validationTransformationName,
                                                    validationTerrainName
                                            ));
                                            try {
                                                List<Object> values = new ArrayList<>();
                                                values.addAll(validationKeyHeaders.stream().map(keys::get).collect(Collectors.toList()));
                                                values.addAll(List.of(validationTransformationName, validationTerrainName));
                                                values.addAll(metrics);
                                                validationPrinter.printRecord(values);
                                                validationPrinter.flush();
                                            } catch (IOException e) {
                                                L.severe(String.format("Cannot save validation results due to %s", e));
                                            }
                                        }
                                    }
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
        try {
            validationPrinter.close(true);
        } catch (IOException e) {
            L.severe(String.format("Cannot close printer for validation results due to %s", e));
        }
    }

    // those three interfaces must be there for the methods that come later
    private interface IODimMapper extends Function<Grid<? extends SensingVoxel>, Pair<Integer, Integer>> {
    }
    private interface RobotMapper extends Function<Grid<? extends SensingVoxel>, Function<Function<double[], double[]>, Robot<?>>> {
    }
    private interface EvolverMapper extends BiFunction<Pair<IODimMapper, RobotMapper>, Grid<? extends SensingVoxel>, Evolver<?, Robot<?>, Double>> {
    }

    // ADD IF FOR DISTRIBUTED
    private static Pair<IODimMapper, RobotMapper> buildRobotMapper(String name) {
        String centralized = "centralized";
        String phases = "phases-(?<f>\\d+(\\.\\d+)?)";
        String distributed = "distributed";

        // ADD HERE IF FOR DISTRIBUTED
        /*
        if (name.matches(distributed)) {
            return Pair.of(
                    body -> Pair.of(DistributedSensing., DistributedSensing.), // i added nOfInputs(body) and nOfOutputs(body) methods and variables in Distibuted sensing in 2hdmsr
                    body -> f -> new Robot<>(
                            new DistributedSensing(SerializationUtils.clone(body), 1),
                            SerializationUtils.clone(body)
                    )
            );
        }

         */
        if (name.matches(centralized)) {
            return Pair.of(
                    body -> Pair.of(CentralizedSensing.nOfInputs(body), CentralizedSensing.nOfOutputs(body)),
                    body -> f -> new Robot<>(
                            new CentralizedSensing(body, f),
                            SerializationUtils.clone(body)
                    )
            );
        }
        if (name.matches(phases)) {
            return Pair.of(
                    body -> Pair.of(2, 1),
                    body -> f -> new Robot<>(
                            new PhaseSin(
                                    -Double.parseDouble(Utils.paramValue(phases, name, "f")),
                                    1d,
                                    Grid.create(
                                            body.getW(),
                                            body.getH(),
                                            (x, y) -> f.apply(new double[]{(double) x / (double) body.getW(), (double) y / (double) body.getH()})[0]
                                    )),
                            SerializationUtils.clone(body)
                    )
            );
        }
        throw new IllegalArgumentException(String.format("Unknown mapper name: %s", name));
    }

    // ok do not touch
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

    // creates a body but not all voxels have all sensors
    private static Grid<? extends SensingVoxel> buildBody(String name) {
        String wbt = "(?<shape>worm|biped|tripod)-(?<w>\\d+)x(?<h>\\d+)-(?<cgp>[tf])-(?<malfunction>[tf])";
        if (name.matches(wbt)) {
            String shape = Utils.paramValue(wbt, name, "shape");
            int w = Integer.parseInt(Utils.paramValue(wbt, name, "w"));
            int h = Integer.parseInt(Utils.paramValue(wbt, name, "h"));
            boolean withCentralPatternGenerator = Utils.paramValue(wbt, name, "cgp").equals("t");
            boolean withMalfunctionSensor = Utils.paramValue(wbt, name, "malfunction").equals("t");
            Grid<? extends SensingVoxel> body = Grid.create(  // i think i should change this i want all voxels with all sensors
                    w, h,
                    (x, y) -> new SensingVoxel(Utils.ofNonNull(
                            new Normalization(new AreaRatio()),
                            withMalfunctionSensor ? new Malfunction() : null,
                            (y == 0) ? new Touch() : null,
                            (y == h - 1) ? new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)) : null,
                            (x == w - 1 && y == h - 1 && withCentralPatternGenerator) ? new Normalization(new TimeFunction(t -> Math.sin(2 * Math.PI * -1 * t), -1, 1)) : null
                    ).stream().filter(Objects::nonNull).collect(Collectors.toList())));
            if (shape.equals("biped")) {
                final Grid<? extends SensingVoxel> finalBody = body;
                body = Grid.create(w, h, (x, y) -> (y == 0 && x > 0 && x < w - 1) ? null : finalBody.get(x, y));
            } else if (shape.equals("tripod")) {
                final Grid<? extends SensingVoxel> finalBody = body;
                body = Grid.create(w, h, (x, y) -> (y != h - 1 && x != 0 && x != w - 1 && x != w / 2) ? null : finalBody.get(x, y));
            }
            return body;
        }
        throw new IllegalArgumentException(String.format("Unknown body name: %s", name));
    }

    // ???? i do not understand what is this method for
    private static EvolverMapper buildEvolverMapper(String name) {
        PartialComparator<Individual<?, Robot<?>, Double>> comparator = PartialComparator.from(Double.class).reversed().comparing(Individual::getFitness);
        String mlpGa = "mlp-(?<h>\\d+(\\.\\d+)?)-ga-(?<nPop>\\d+)";
        String mlpGaDiv = "mlp-(?<h>\\d+(\\.\\d+)?)-gadiv-(?<nPop>\\d+)";
        String mlpCmaEs = "mlp-(?<h>\\d+(\\.\\d+)?)-cmaes";
        String graphea = "fgraph-hash-speciated-(?<nPop>\\d+)";
        if (name.matches(mlpGa)) {
            double ratioOfFirstLayer = Double.parseDouble(Utils.paramValue(mlpGa, name, "h"));
            int nPop = Integer.parseInt(Utils.paramValue(mlpGa, name, "nPop"));
            return (p, body) -> new StandardEvolver<>(
                    ((Function<List<Double>, MultiLayerPerceptron>) ws -> new MultiLayerPerceptron(
                            MultiLayerPerceptron.ActivationFunction.TANH,
                            p.first().apply(body).first(),
                            ratioOfFirstLayer == 0 ? new int[0] : new int[]{(int) Math.round(p.first().apply(body).first() * ratioOfFirstLayer)},
                            p.first().apply(body).second(),
                            ws.stream().mapToDouble(d -> d).toArray()
                    )).andThen(mlp -> p.second().apply(body).apply(mlp)),
                    new FixedLengthListFactory<>(
                            MultiLayerPerceptron.countWeights(
                                    p.first().apply(body).first(),
                                    ratioOfFirstLayer == 0 ? new int[0] : new int[]{(int) Math.round(p.first().apply(body).first() * ratioOfFirstLayer)},
                                    p.first().apply(body).second()
                            ),
                            new UniformDoubleFactory(-1, 1)
                    ),
                    comparator,
                    nPop,
                    Map.of(
                            new GaussianMutation(1d), 0.2d,
                            new GeometricCrossover(), 0.8d
                    ),
                    new Tournament(5),
                    new Worst(),
                    nPop,
                    true
            );
        }
        if (name.matches(mlpGaDiv)) {
            double ratioOfFirstLayer = Double.parseDouble(Utils.paramValue(mlpGaDiv, name, "h"));
            int nPop = Integer.parseInt(Utils.paramValue(mlpGaDiv, name, "nPop"));
            return (p, body) -> new StandardWithEnforcedDiversityEvolver<>(
                    ((Function<List<Double>, MultiLayerPerceptron>) ws -> new MultiLayerPerceptron(
                            MultiLayerPerceptron.ActivationFunction.TANH,
                            p.first().apply(body).first(),
                            ratioOfFirstLayer == 0 ? new int[0] : new int[]{(int) Math.round(p.first().apply(body).first() * ratioOfFirstLayer)},
                            p.first().apply(body).second(),
                            ws.stream().mapToDouble(d -> d).toArray()
                    )).andThen(mlp -> p.second().apply(body).apply(mlp)),
                    new FixedLengthListFactory<>(
                            MultiLayerPerceptron.countWeights(
                                    p.first().apply(body).first(),
                                    ratioOfFirstLayer == 0 ? new int[0] : new int[]{(int) Math.round(p.first().apply(body).first() * ratioOfFirstLayer)},
                                    p.first().apply(body).second()
                            ),
                            new UniformDoubleFactory(-1, 1)
                    ),
                    comparator,
                    nPop,
                    Map.of(
                            new GaussianMutation(1d), 0.2d,
                            new GeometricCrossover(), 0.8d
                    ),
                    new Tournament(5),
                    new Worst(),
                    nPop,
                    true,
                    100
            );
        }
        if (name.matches(mlpCmaEs)) {
            double ratioOfFirstLayer = Double.parseDouble(Utils.paramValue(mlpCmaEs, name, "h"));
            return (p, body) -> new CMAESEvolver<>(
                    ((Function<List<Double>, MultiLayerPerceptron>) ws -> new MultiLayerPerceptron(
                            MultiLayerPerceptron.ActivationFunction.TANH,
                            p.first().apply(body).first(),
                            ratioOfFirstLayer == 0 ? new int[0] : new int[]{(int) Math.round(p.first().apply(body).first() * ratioOfFirstLayer)},
                            p.first().apply(body).second(),
                            ws.stream().mapToDouble(d -> d).toArray()
                    )).andThen(mlp -> p.second().apply(body).apply(mlp)),
                    new FixedLengthListFactory<>(
                            MultiLayerPerceptron.countWeights(
                                    p.first().apply(body).first(),
                                    ratioOfFirstLayer == 0 ? new int[0] : new int[]{(int) Math.round(p.first().apply(body).first() * ratioOfFirstLayer)},
                                    p.first().apply(body).second()
                            ),
                            new UniformDoubleFactory(-1, 1)
                    ),
                    comparator,
                    -1,
                    1
            );
        }
        if (name.matches(graphea)) {
            int nPop = Integer.parseInt(Utils.paramValue(graphea, name, "nPop"));
            return (p, body) -> new SpeciatedEvolver<>(
                    GraphUtils.mapper((Function<IndexedNode<Node>, Node>) IndexedNode::content, (Function<Collection<Double>, Double>) Misc::first)
                            .andThen(FunctionGraph.builder())
                            .andThen(fg -> p.second().apply(body).apply(fg)),
                    new ShallowSparseFactory(
                            0d, 0d, 1d,
                            p.first().apply(body).first(),
                            p.first().apply(body).second()
                    ).then(GraphUtils.mapper(IndexedNode.incrementerMapper(Node.class), Misc::first)),
                    comparator,
                    nPop,
                    Map.of(
                            new IndexedNodeAddition<>(
                                    FunctionNode.sequentialIndexFactory(BaseFunction.TANH),
                                    n -> n.getFunction().hashCode(),
                                    p.first().apply(body).first() + p.first().apply(body).second() + 1,
                                    (w, r) -> w,
                                    (w, r) -> r.nextGaussian()
                            ), 1d,
                            new ArcModification<>((w, r) -> w + r.nextGaussian(), 1d), 1d,
                            new ArcAddition
                                    <>(Random::nextGaussian, false), 3d,
                            new AlignedCrossover<>(
                                    (w1, w2, r) -> w1 + (w2 - w1) * (r.nextDouble() * 3d - 1d),
                                    node -> node.content() instanceof Output,
                                    false
                            ), 1d
                    ),
                    5,
                    (new Jaccard()).on(i -> i.getGenotype().nodes()),
                    0.25,
                    individuals -> {
                        double[] fitnesses = individuals.stream().mapToDouble(Individual::getFitness).toArray();
                        Individual<Graph<IndexedNode<Node>, Double>, Robot<?>, Double> r = Misc.first(individuals);
                        return new Individual<>(
                                r.getGenotype(),
                                r.getSolution(),
                                Misc.median(fitnesses),
                                r.getBirthIteration()
                        );
                    },
                    0.75
            );
        }
        throw new IllegalArgumentException(String.format("Unknown evolver name: %s", name));
    }

}


// VECCHIO
/*

public class MainCoEvo extends Worker {

    public MainCoEvo(String[] args) throws FileNotFoundException {
        super(args);
    }

    public static void main(String[] args) throws FileNotFoundException {
        new MainCoEvo(args);
    }

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
        int nPop = i(a("npop", "10"));                                 // quanti robottini simulo
        int iterations = i(a("iterations", "20"));                    // quante generazioni faccio
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
                L.log(Level.SEVERE, String.format("Cannot read builder properties due to %s", ex), ex);
            } catch (FileNotFoundException ex) {
                L.log(Level.SEVERE, String.format("Cannot wrtie builder properties on file due to %s", ex), ex);
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
                                    Grid<Boolean> body = null;       // le cose con il 2 sono con la grid quelle senza con la sequence
                                    double[] controller = null;

                                    Pair<Grid<Boolean>, double[]> pair;
                                    pair = Pair.build(body, controller);

                                    Factory<Pair<Grid<Boolean>, double[]>> factory = null;
                                    NonDeterministicFunction<Pair<Grid<Boolean>, double[]>, Robot.Description> mapper = null;

                                    factory = new PairSequenceFactory(-Math.PI, Math.PI, width, height);
                                    mapper = Mapper.getPhasesMapper(builder, drivingFrequency);

                                    //prepare evolver  PENSO SIA OK
                                    Evolver<Pair<Grid<Boolean>, double[]>, Robot.Description, List<Double>> evolver = null;

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
                                        Crossover<Pair<Grid<Boolean>, double[]>> crossover;
                                        crossover = new PairUniformCrossover(Range.closedOpen(-1d, 2d));

                                        Mutation<Pair<Grid<Boolean>, double[]>> mutation;
                                        mutation = new PairGaussianMutation(mutationSigma);

                                        Map<GeneticOperator<Pair<Grid<Boolean>, double[]>>, Double> operators = new LinkedHashMap<>();

                                        if (evolverName.split("-")[1].equals("1op")) {
                                            operators.put(crossover.andThen(mutation), 1d);
                                        } else if (evolverName.split("-")[1].equals("2op")) {
                                            operators.put(crossover, 0.8d);
                                            operators.put(mutation, 0.2d);
                                        }
                                        if (evolverName.startsWith("standardDiv")) {
                                            evolver = new StandardWithEnforcedDiversity<Pair<Grid<Boolean>, double[]>, Robot.Description, List<Double>>(
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
                                            evolver = new StandardEvolver<Pair<Grid<Boolean>, double[]>, Robot.Description, List<Double>>(
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
                                        L.log(Level.SEVERE, String.format("Cannot solve problem: %s", ex), ex);
                                    }
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

/*

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


public class MainCoEvo  extends Worker {   // per il controllore a rete neurale distribuita

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
