package org.usfirst.frc.team1806.robot.util;

import java.util.LinkedList;

public class SamplingFilter {

    private LinkedList<SamplingFilterValue> samplingBuffer;
    int maxSamples;

    /**
     * Uses a {@link LinkedList} to sample a double value and average the last n samples.
     * @param numberOfSamples number of samples to average.
     */
    public SamplingFilter(int numberOfSamples){
        samplingBuffer = new LinkedList<SamplingFilterValue>();
        maxSamples = numberOfSamples;
    }

    /**
     * Updates the sample list and returns the current average.
     * @param sample the current sample to add to the list to filter.
     * @return {@link Double}- The current sample to add to the list.
     */
    public Double update(Double sample){
        Double currentRunningTotal = samplingBuffer.getLast().getRunningTotal();
        if(samplingBuffer.size() >= maxSamples)
        {
            currentRunningTotal -= samplingBuffer.getFirst().getCurrentValue();
            samplingBuffer.removeFirst();
        }

        currentRunningTotal += sample;
        samplingBuffer.add(new SamplingFilterValue(sample, currentRunningTotal));
        return currentRunningTotal / samplingBuffer.size();
    }

    /**
     *
     * @return The current average value of the samples.
     */
    public Double getCurrentAverage(){
        return samplingBuffer.getLast().getRunningTotal() / samplingBuffer.size();
    }

    /**
     * Clears the sample list so it can start averaging from no samples.
     */
    public void clear(){
        samplingBuffer.clear();
    }



}
