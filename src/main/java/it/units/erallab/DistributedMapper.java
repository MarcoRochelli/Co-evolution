package it.units.erallab;

import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.Sensor;

import java.util.List;
import java.util.function.Function;

public class DistributedMapper implements Function<List<Double>, Robot<SensingVoxel>> {
    private final int width;
    private final int height;
    private final List<Sensor> sensors;
    private final int[] innerNeurons;
    private final int signals;


    public DistributedMapper(int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        this.width = width;
        this.height = height;
        this.sensors = sensors;
        this.innerNeurons = innerNeurons;
        this.signals = signals;
    }

    // duplicate sensors with
    @Override
    public Robot<SensingVoxel> apply(List<Double> doubles) {
        int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum();
        int nOfOutputs = signals * 4 + 1;
        int nOfWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));
        if (doubles.size() != width * height + nOfWeights){
            throw
        }
        // body
        // controls of connection
        // controller
        // create array of double weights
            return null;
    }
}
