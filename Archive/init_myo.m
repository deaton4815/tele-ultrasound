function hMyo = init_myo()
% Initialize Myo interface through MiniVIE

hMyo = Inputs.MyoUdp.getInstance();
hMyo.initialize();

disp('[init_myo] Myo initialized');
end
