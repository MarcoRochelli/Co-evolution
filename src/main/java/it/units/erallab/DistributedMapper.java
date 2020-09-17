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
import org.apache.commons.lang3.SerializationUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.function.Function;
import java.util.stream.IntStream;

public class DistributedMapper {

    public static Function<List<Double>, Robot<SensingVoxel>>
    distributedMapper(int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals, List<Double> doubles) {
        Random random = new Random();
        final double threshold = 0d;

        SensingVoxel sensingVoxel = new SensingVoxel(List.of(  // clone this with SerializationUtils.clone(sensingVoxel)
                new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
                new Normalization(new AreaRatio())
        ));


        int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum();
        int nOfOutputs = signals * 4 + 1;
        int nOfWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));
        if (doubles.size() != width * height + nOfWeights) { // correct length of the genotype
            throw new IllegalArgumentException("Sensor list has wrong dimension");


            int c = 0;
            Grid<SensingVoxel> body = Grid.create(width, height);
            for (double entry : doubles) {
                if (c < width * height) {  // < or <=???????
                    if (entry > threshold) {
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
            // i think ok until here


            // creates a distributed controller
            DistributedSensing distributedSensing = new DistributedSensing(SerializationUtils.clone(body), 1);
            for (Grid.Entry<SensingVoxel> entry : body) {
                MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                        MultiLayerPerceptron.ActivationFunction.TANH,
                        distributedSensing.nOfInputs(entry.getX(), entry.getY()),
                        new int[0],
                        distributedSensing.nOfOutputs(entry.getX(), entry.getY())
                );
                double[] ws = mlp.getParams();
                IntStream.range(0, ws.length).forEach(i -> ws[i] = random.nextGaussian());
                mlp.setParams(ws);
                distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp);
            }
            // creates sensing distributed robot
            Robot<SensingVoxel> robot = new Robot<>(
                    distributedSensing,
                    SerializationUtils.clone(body)
            );



        }
    }

        // to test the mapper
        public static void main (String[]args){

            // problem to solve
            Locomotion locomotion = new Locomotion(
                    40,
                    Locomotion.createTerrain("flat"), // se uneven deve esserci qualcosa dopo es. uneven5; uneven-qualcosa da errore
                    // oppure uneven5 al posto di 5 posso mettere altri numeri più è alto più è in discesa il terreno
                    Lists.newArrayList(Locomotion.Metric.TRAVEL_X_VELOCITY),
                    new Settings()
            );

            List<Double> values = new ArrayList<Double>();
            values.add(2d);
            values.add(2d);
            values.add(2d);

            values.add(2d);
            values.add(2d);
            values.add(2d);


            Function<List<Double>, Robot<SensingVoxel>> mapper;
            mapper = DistributedMapper.
                    // this is how to use a function
                    Robot < SensingVoxel > mostro = mapper.apply(// what should i put here);


                    Grid < Pair < String, Robot < ? >>> namedSolutionGrid = Grid.create(1, 1);// i create a one for one graphic
            namedSolutionGrid.set(0, 0, Pair.of("mostro", mostro));        // where i put mostro
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


/*
        public class DistributedMapper implements Function<List<Double>, Robot<SensingVoxel>> {



            private final int width;
            private final int height;
            private final List<Sensor> sensors;
            private final int[] innerNeurons;
            private final int signals;
            Random random = new Random();
            private final double threshold = 0d;

            SensingVoxel sensingVoxel = new SensingVoxel(List.of(  // clone this with SerializationUtils.clone(sensingVoxel)
                    new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
                    new Normalization(new AreaRatio())
            ));


            public DistributedMapper(int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
                this.width = width;
                this.height = height;
                this.sensors = sensors;
                this.innerNeurons = innerNeurons;
                this.signals = signals;
            }


            @Override
                public Robot<SensingVoxel> apply (List < Double > doubles) { // doubles is the genotype
                    int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum();
                    int nOfOutputs = signals * 4 + 1;
                    int nOfWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));
                    if (doubles.size() != width * height + nOfWeights) { // correct length of the genotype
                        throw new IllegalArgumentException("Sensor list has wrong dimension");
                    }

                    int c = 0;
                    Grid<SensingVoxel> body = Grid.create(width, height);
                    for (double entry : doubles) {
                        if (c < width * height) {  // < or <=???????
                            if (entry > threshold) {
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
                    // i think ok until here


                    // creates a distributed controller
                    DistributedSensing distributedSensing = new DistributedSensing(SerializationUtils.clone(body), 1);
                    for (Grid.Entry<SensingVoxel> entry : body) {
                        MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                                MultiLayerPerceptron.ActivationFunction.TANH,
                                distributedSensing.nOfInputs(entry.getX(), entry.getY()),
                                new int[0],
                                distributedSensing.nOfOutputs(entry.getX(), entry.getY())
                        );
                        double[] ws = mlp.getParams();
                        IntStream.range(0, ws.length).forEach(i -> ws[i] = random.nextGaussian());
                        mlp.setParams(ws);
                        distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp);
                    }
                    // creates sensing distributed robot
                    Robot<SensingVoxel> robot = new Robot<>(
                            distributedSensing,
                            SerializationUtils.clone(body)
                    );


                    // controller
                    // create array of double weights
                    return null;
                }

            }
        }
         */