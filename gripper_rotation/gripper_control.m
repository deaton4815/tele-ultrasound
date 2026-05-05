function grip = gripper_control(hMyo, training_data)
    hData = PatternRecognition.TrainingData();
    hData.loadTrainingData(training_data);

    hLda = SignalAnalysis.Lda;
    hLda.initialize(hData);
    hLda.train();
    hLda.computeError();
    
    % Class names
    classNames = hLda.getClassNames;
    disp(classNames);
        
    gripMin = 0.005;
    gripMax = 0.03;
    gripStep = 0.003;
    grip = gripMin;

    emgData = hMyo.getData(hLda.NumSamplesPerWindow,1:8);
    features2D = hLda.extractfeatures(emgData);
    [classDecision, voteDecision] = hLda.classify(reshape(features2D',[],1));
    className = classNames{classDecision};

    if contains(lower(className), 'open')
        grip = min(grip + gripStep, gripMax);
    else
        grip = max(grip - gripStep, gripMin);
    end
end