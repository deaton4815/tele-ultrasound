function myo_training
    cd('C:\GitHub\MiniVIE');
    MiniVIE.configurePath;
    
    hMyo = Inputs.MyoUdp.getInstance();
    hMyo.initialize();
    cd('C:\GitHub\hrilabs\Lab2_EMGInterfacing');
    obj = MiniVIE;
end
