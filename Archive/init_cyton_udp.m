function [udpAction, udpUnity] = init_cyton_udp(useAction, useUnity)
% Initialize UDP interfaces for Actin Viewer and Unity vCyton

udpAction = [];
udpUnity = [];

if useAction
    udpAction = PnetClass(8889, 8888, '127.0.0.1');
    udpAction.initialize();
    disp('[init_cyton_udp] Actin UDP initialized');
end

if useUnity
    udpUnity = PnetClass(12002, 12001, '127.0.0.1');
    udpUnity.initialize();
    disp('[init_cyton_udp] Unity UDP initialized');
end
end
