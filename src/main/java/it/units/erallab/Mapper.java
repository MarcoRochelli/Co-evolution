package it.units.erallab;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.*;
import it.units.erallab.hmsrobots.tasks.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Util;
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import org.apache.commons.lang3.SerializationUtils;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;

import java.util.List;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.stream.IntStream;

public class Mapper {

    /* // should i do it with a function???????
    Yes of course but better to wait Eric to be sure


     */

    public static Robot<SensingVoxel> distributedMapper(double[] structure, int width, int height) {
        //Settings settings = new Settings();
        //settings.setStepFrequency(1d / 30d);
        Random random = new Random();
        double threshold = 0d;

        // trasformare l'array in una grid
        int c = 0;
        Grid<Boolean> body = Grid.create(width, height);
        for (double entry : structure) {
            if (entry > threshold) {
                body.set(c % width, c / width, true);
            } else {
                body.set(c % width, c / width, false);
            }
            c = c + 1;
        }

        // checks if the robot is empty
        Grid<Boolean> emptyBody = Grid.create(body.getW(), body.getH(), false); // creates an empty grid
        if (body.equals(emptyBody)) {
            body.set(0, 0, Boolean.TRUE); // if the body is empty puts a voxel (0,0)
        }

        // checks if the robot is connected
        body = Util.gridLargestConnected(body, i -> i);
        for (Grid.Entry<Boolean> entry : body) {
            if (entry.getValue() == null) {
                body.set(entry.getX(), entry.getY(), false);
            }
        }

        // crea una griglia di voxel con sensori
        Grid<Boolean> finalBody = body;
        Grid<SensingVoxel> voxels = Grid.create(body.getW(), body.getH(), (x, y) -> {
            if (finalBody.get(x, y)) {
                return new SensingVoxel(List.of( // WHICH ARE THE SENSORS here i put just some
                        // serializationUtils.sensor clone sensors
                        /*
                        new TimeFunction(t -> Math.sin(2 * Math.PI * t), -1d, 1d),
                        new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y),
                        new Average(new Velocity(true, 3d, Velocity.Axis.X, Velocity.Axis.Y), 1d),
                        new Average(new Touch(), 1d),
                        new AreaRatio(),
                        new ControlPower(settings.getStepFrequency()),

                         */
                        new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)),
                        new Normalization(new AreaRatio())
                        //
                ));
            }
            return null;
        });

        // there is a method that returns dimension of
        // crea un controllore distribuito
        DistributedSensing distributedSensing = new DistributedSensing(SerializationUtils.clone(voxels), 1);
        for (Grid.Entry<SensingVoxel> entry : voxels) {
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
        // crea il robot sensing distribuito
        Robot<SensingVoxel> robot = new Robot<>(
                distributedSensing,
                SerializationUtils.clone(voxels)
        );
        return robot;
    }

    // to test the mapper
    public static void main(String[] args) {
        // creo il prblema da risolvere
        Locomotion locomotion = new Locomotion(
                40,
                Locomotion.createTerrain("flat"), // se uneven deve esserci qualcosa dopo es. uneven5; uneven-qualcosa da errore
                // oppure uneven5 al posto di 5 posso mettere altri numeri più è alto più è in discesa il terreno
                Lists.newArrayList(Locomotion.Metric.TRAVEL_X_VELOCITY),
                new Settings()
        );

        double[] body = new double[]{2d, 2d, 2d, 0d, 0d, 0d, 0d, 0d, 2d, 2d, 2d, 2d};
        Robot<SensingVoxel> mostro = distributedMapper(body, 4, 3);


        Grid<Pair<String, Robot<?>>> namedSolutionGrid = Grid.create(1, 1);// i create a one for one graphic
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
