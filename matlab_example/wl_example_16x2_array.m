%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% wl_example_8x2_array.m
%
% Compatibility:
%   WARPLab:    v7.1.0 and later
%   Hardware:   v2 and v3
%
% Description:
%   See warpproject.org/trac/wiki/WARPLab/Examples/8x2Array
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
figure(1);clf;

USE_AGC = true;
RUN_CONTINOUSLY = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Set up the WARPLab experiment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Create a vector of node objects
%This experiment uses 5 nodes: 4 will act as a transmitter and 1 will act
%as a receiver.
%   nodes(1): 1st transmitter
%   nodes(2): 2nd transmitter (receives clocks and triggers from
%             1st transmittter)
%   nodes(3): 3rd transmitter (receives clocks and triggers from
%             2nd transmitter)
%   nodes(4): 4th transmitter (receives clocks and triggers from
%             3rd transmitter) 
%   nodes(5): Receiver
nodes = wl_initNodes(5);

%Create a UDP broadcast trigger and tell each node to be ready for it
eth_trig = wl_trigger_eth_udp_broadcast;
wl_triggerManagerCmd(nodes,'add_ethernet_trigger',[eth_trig]);

%Read Trigger IDs into workspace
[T_IN_ETH,T_IN_ENERGY,T_IN_AGCDONE,T_IN_REG,T_IN_D0,T_IN_D1,T_IN_D2,T_IN_D3] =  wl_getTriggerInputIDs(nodes(1));
[T_OUT_BASEBAND, T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D2, T_OUT_D3] = wl_getTriggerOutputIDs(nodes(1));
[T_IN_ETH,T_IN_ENERGY,T_IN_AGCDONE,T_IN_REG,T_IN_D0,T_IN_D1,T_IN_D2,T_IN_D3] =  wl_getTriggerInputIDs(nodes(2));
[T_OUT_BASEBAND, T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D2, T_OUT_D3] = wl_getTriggerOutputIDs(nodes(2));
[T_IN_ETH,T_IN_ENERGY,T_IN_AGCDONE,T_IN_REG,T_IN_D0,T_IN_D1,T_IN_D2,T_IN_D3] =  wl_getTriggerInputIDs(nodes(3));
[T_OUT_BASEBAND, T_OUT_AGC, T_OUT_D0, T_OUT_D1, T_OUT_D2, T_OUT_D3] = wl_getTriggerOutputIDs(nodes(3));

%For the 1st, 2nd, 3rd transmit node, we will allow Ethernet to trigger the buffer
%baseband, the AGC, and debug0 (which is mapped to pin 8 on the debug
%header). We also will allow Ethernet to trigger the same signals for the 
%receiving node.
wl_triggerManagerCmd([nodes(1),nodes(2),nodes(3),nodes(5)],'output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC,T_OUT_D0],[T_IN_ETH,T_IN_REG]);

%For the receive node, we will allow debug3 (mapped to pin 15 on the
%debug header) to trigger the buffer baseband, and the AGC
nodes(2).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC],[T_IN_D3]);
nodes(3).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC],[T_IN_D3]);
nodes(4).wl_triggerManagerCmd('output_config_input_selection',[T_OUT_BASEBAND,T_OUT_AGC],[T_IN_D3]);

%For the receive node, we enable the debounce circuity on the debug 3 input
%to deal with the fact that the signal may be noisy.
nodes(2).wl_triggerManagerCmd('input_config_debounce_mode',[T_IN_D3],'enable'); 
nodes(3).wl_triggerManagerCmd('input_config_debounce_mode',[T_IN_D3],'enable');
nodes(4).wl_triggerManagerCmd('input_config_debounce_mode',[T_IN_D3],'enable');

%Since the debounce circuitry is enabled, there will be a delay at the
%receiver node for its input trigger. To better align the transmitter and
%receiver, we can artificially delay the transmitters trigger outputs that
%drive the buffer baseband and the AGC.
%
%NOTE:  Due to HW changes in WARPLab 7.2.0, the input delay of the trigger 
%manager increased by one clock cycle;  Therefore, when using WARPLab 7.2.0, 
%we need to increase the output delay by one step.  If using WARPLab 7.1.0, 
%please use the commented out line below:
%
%nodes(1).wl_triggerManagerCmd('output_config_delay',[T_OUT_BASEBAND,T_OUT_AGC],[50]); %50ns delay  - WARPLab 7.1.0
%
nodes(1).wl_triggerManagerCmd('output_config_delay',[T_OUT_BASEBAND,T_OUT_AGC],[56.25]); %56.25ns delay  - WARPLab 7.2.0
nodes(2).wl_triggerManagerCmd('output_config_delay',[T_OUT_BASEBAND,T_OUT_AGC],[56.25]);
nodes(3).wl_triggerManagerCmd('output_config_delay',[T_OUT_BASEBAND,T_OUT_AGC],[56.25]);

%Get IDs for the interfaces on the boards. Since this example assumes each
%board has the same interface capabilities, we only need to get the IDs
%from one of the boards
[RFA,RFB,RFC,RFD] = wl_getInterfaceIDs(nodes(1));

%Set up the interface for the experiment
wl_interfaceCmd(nodes,'RF_ALL','tx_gains',3,30);
wl_interfaceCmd(nodes,'RF_ALL','channel',2.4,11);

if(USE_AGC)
    wl_interfaceCmd(nodes,'RF_ALL','rx_gain_mode','automatic');
    wl_basebandCmd(nodes,'agc_target',-8);
    wl_basebandCmd(nodes,'agc_trig_delay', 511);
else
    wl_interfaceCmd(nodes,'RF_ALL','rx_gain_mode','manual');
    RxGainRF = 1; %Rx RF Gain in [1:3]
    RxGainBB = 4; %Rx Baseband Gain in [0:31]
    wl_interfaceCmd(nodes,'RF_ALL','rx_gains',RxGainRF,RxGainBB);
end

wl_interfaceCmd(nodes,'RF_ALL','tx_lpf_corn_freq',2); %Configure Tx for 36MHz of bandwidth
wl_interfaceCmd(nodes,'RF_ALL','rx_lpf_corn_freq',3); %Configure Rx for 36MHz of bandwidth

%We'll use the transmitter's I/Q buffer size to determine how long our
%transmission can be
txLength = nodes(5).baseband.txIQLen;

%Set up the baseband for the experiment
wl_basebandCmd(nodes,'tx_delay',0);
wl_basebandCmd(nodes,'tx_length',txLength);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Signal processing to generate transmit signal
% Here, we can send any signal we want out of each of the 8 transmit 
% antennas. For visualization, we'll send "pink" noise of 1MHz out of 
% each, but centered at different parts of the 40MHz band.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% First generate the preamble for AGC. The preamble corresponds to the
% short symbols from the 802.11a PHY standard
shortSymbol_freq = [0 0 0 0 0 0 0 0 1+i 0 0 0 -1+i 0 0 0 -1-i 0 0 0 1-i 0 0 0 -1-i 0 0 0 1-i 0 0 0 0 0 0 0 1-i 0 0 0 -1-i 0 0 0 1-i 0 0 0 -1-i 0 0 0 -1+i 0 0 0 1+i 0 0 0 0 0 0 0].';
shortSymbol_freq = [zeros(32,1);shortSymbol_freq;zeros(32,1)];
shortSymbol_time = ifft(fftshift(shortSymbol_freq));
shortSymbol_time = (shortSymbol_time(1:32).')./max(abs(shortSymbol_time));
shortsyms_rep = repmat(shortSymbol_time,1,30);

preamble_single = shortsyms_rep;
preamble_single = preamble_single(:);

num_antenna = 4;

shifts = floor(linspace(0,31,num_antenna));
for k = 1:num_antenna
   %Shift preamble for each antenna so we don't have accidental beamforming
   preamble(:,k) = circshift(preamble_single,shifts(k));
end

Ts = 1/(wl_basebandCmd(nodes(5),'tx_buff_clk_freq'));

payload = complex(randn(txLength-length(preamble),num_antenna),randn(txLength-length(preamble),num_antenna));
payload_freq = fftshift(fft(payload));
freqVec = linspace(-((1/Ts)/2e6),((1/Ts)/2e6),txLength-length(preamble));
BW = 1; %MHz 
noise_centerFreqs = linspace(-12,12,num_antenna);
for k = 1:num_antenna
    payload_freq((freqVec < (noise_centerFreqs(k) - BW/2)) | (freqVec > (noise_centerFreqs(k) + BW/2)),k)=0;
end
payload = ifft(fftshift(payload_freq));

txData = [preamble;payload];

node_rx1 = nodes(1);
node_rx2 = nodes(2);
node_rx3 = nodes(3);
node_rx4 = nodes(4);
node_tx = nodes(5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Transmit and receive signal using WARPLab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wl_basebandCmd(node_tx,[RFA,RFB,RFC,RFD], 'write_IQ', txData(:,1:4)); %First 4 columns of txData is for 1st tx
%wl_basebandCmd(node_tx2,[RFA,RFB,RFC,RFD], 'write_IQ', txData(:,5:8)); %Second 4 columns of txData is for 2nd tx
%wl_basebandCmd(node_tx3,[RFA,RFB,RFC,RFD], 'write_IQ', txData(:,9:12)); %Third 4 columns of txData is for 3rd tx
%wl_basebandCmd(node_tx4,[RFA,RFB,RFC,RFD], 'write_IQ', txData(:,13:16)); %Fourth 4 columns of txData is for 4th tx

wl_basebandCmd(node_rx1,'RF_ALL','rx_buff_en');
wl_basebandCmd(node_rx2,'RF_ALL','rx_buff_en');
wl_basebandCmd(node_rx3,'RF_ALL','rx_buff_en');
wl_basebandCmd(node_rx4,'RF_ALL','rx_buff_en');
wl_basebandCmd(node_tx,'RF_ALL','tx_buff_en');

wl_interfaceCmd(node_rx1,'RF_ALL','rx_en');
wl_interfaceCmd(node_rx2,'RF_ALL','rx_en');
wl_interfaceCmd(node_rx3,'RF_ALL','rx_en');
wl_interfaceCmd(node_rx4,'RF_ALL','rx_en');
wl_interfaceCmd(node_tx,'RF_ALL','tx_en');


set(gcf, 'KeyPressFcn','RUN_CONTINOUSLY=0;');
fprintf('Press any key to halt experiment\n')

while(1)
    eth_trig.send();

    rx_IQ1 = wl_basebandCmd(node_rx1,[RFA,RFB,RFC,RFD],'read_IQ', 0, txLength);
    rx_IQ2 = wl_basebandCmd(node_rx2,[RFA,RFB,RFC,RFD],'read_IQ', 0, txLength);
    rx_IQ3 = wl_basebandCmd(node_rx3,[RFA,RFB,RFC,RFD],'read_IQ', 0, txLength);
    rx_IQ4 = wl_basebandCmd(node_rx4,[RFA,RFB,RFC,RFD],'read_IQ', 0, txLength);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualize results
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    t = [0:Ts:(txLength-1)*Ts].';
    figure(1);
    ax(1) = subplot(8,8,1);
    plot(t,real(rx_IQ1(:,1)))
    title('Node1: Re\{rx\_IQ_{RFA}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(8,8,2);
    plot(t,real(rx_IQ1(:,2)))
    title('Node1: Re\{rx\_IQ_{RFB}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    FFTSIZE = 1024;

    ax(1) = subplot(8,8,3);
    rx_IQ_slice = rx_IQ1(2049:end,1);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node1: FFT Magnitude of rx\_IQ_{RFA}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])
    
    ax(2) = subplot(8,8,4);
    rx_IQ_slice = rx_IQ1(2049:end,2);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node1: FFT Magnitude of rx\_IQ_{RFB}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])
    
    ax(1) = subplot(8,8,5);
    plot(t,real(rx_IQ1(:,3)))
    title('Node1: Re\{rx\_IQ_{RFC}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(8,8,6);
    plot(t,real(rx_IQ1(:,4)))
    title('Node1: Re\{rx\_IQ_{RFD}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    ax(1) = subplot(8,8,7);
    rx_IQ_slice = rx_IQ1(2049:end,3);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node1: FFT Magnitude of rx\_IQ_{RFC}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])
    
    ax(2) = subplot(8,8,8);
    rx_IQ_slice = rx_IQ1(2049:end,4);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node1: FFT Magnitude of rx\_IQ_{RFD}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])
  
    ax(1) = subplot(8,8,9);
    plot(t,real(rx_IQ2(:,1)))
    title('Node2: Re\{rx\_IQ_{RFA}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(8,8,10);
    plot(t,real(rx_IQ2(:,2)))
    title('Node2: Re\{rx\_IQ_{RFB}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    ax(1) = subplot(8,8,11);
    rx_IQ_slice = rx_IQ2(2049:end,1);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node2: FFT Magnitude of rx\_IQ_{RFA}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])

    ax(2) = subplot(8,8,12);
    rx_IQ_slice = rx_IQ2(2049:end,2);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node2: FFT Magnitude of rx\_IQ_{RFB}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])

    ax(1) = subplot(8,8,13);
    plot(t,real(rx_IQ2(:,3)))
    title('Node2: Re\{rx\_IQ_{RFC}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(8,8,14);
    plot(t,real(rx_IQ2(:,4)))
    title('Node2: Re\{rx\_IQ_{RFD}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    ax(1) = subplot(8,8,15);
    rx_IQ_slice = rx_IQ2(2049:end,3);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node2: FFT Magnitude of rx\_IQ_{RFC}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])

    ax(2) = subplot(8,8,16);
    rx_IQ_slice = rx_IQ2(2049:end,4);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node2: FFT Magnitude of rx\_IQ_{RFD}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])

    ax(1) = subplot(8,8,17);
    plot(t,real(rx_IQ3(:,1)))
    title('Node3: Re\{rx\_IQ_{RFA}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(8,8,18);
    plot(t,real(rx_IQ3(:,2)))
    title('Node3: Re\{rx\_IQ_{RFB}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    ax(1) = subplot(8,8,19);
    rx_IQ_slice = rx_IQ3(2049:end,1);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node3: FFT Magnitude of rx\_IQ_{RFA}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])

    ax(2) = subplot(8,8,20);
    rx_IQ_slice = rx_IQ3(2049:end,2);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node3: FFT Magnitude of rx\_IQ_{RFB}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])

    ax(1) = subplot(8,8,21);
    plot(t,real(rx_IQ3(:,3)))
    title('Node3: Re\{rx\_IQ_{RFC}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(8,8,22);
    plot(t,real(rx_IQ3(:,4)))
    title('Node3: Re\{rx\_IQ_{RFD}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    ax(1) = subplot(8,8,23);
    rx_IQ_slice = rx_IQ3(2049:end,3);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node3: FFT Magnitude of rx\_IQ_{RFC}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])

    ax(2) = subplot(8,8,24);
    rx_IQ_slice = rx_IQ3(2049:end,4);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node3: FFT Magnitude of rx\_IQ_{RFD}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])

    ax(1) = subplot(8,8,25);
    plot(t,real(rx_IQ4(:,1)))
    title('Node4: Re\{rx\_IQ_{RFA}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(8,8,26);
    plot(t,real(rx_IQ4(:,2)))
    title('Node4: Re\{rx\_IQ_{RFB}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    ax(1) = subplot(8,8,27);
    rx_IQ_slice = rx_IQ4(2049:end,1);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node4: FFT Magnitude of rx\_IQ_{RFA}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])

    ax(2) = subplot(8,8,28);
    rx_IQ_slice = rx_IQ4(2049:end,2);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node4: FFT Magnitude of rx\_IQ_{RFB}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])

    ax(1) = subplot(8,8,29);
    plot(t,real(rx_IQ4(:,3)))
    title('Node4: Re\{rx\_IQ_{RFC}\}')
    xlabel('Time (s)')
    axis([0, max(t),-1,1])
    ax(2) = subplot(8,8,30);
    plot(t,real(rx_IQ4(:,4)))
    title('Node4: Re\{rx\_IQ_{RFD}\}')
    xlabel('Time (s)')
    %linkaxes(ax,'x')
    axis([0, max(t),-1,1])

    ax(1) = subplot(8,8,31);
    rx_IQ_slice = rx_IQ4(2049:end,3);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node4: FFT Magnitude of rx\_IQ_{RFC}')
    xlabel('Frequency (MHz)')
    axis([-20, 20,-20,40])

    ax(2) = subplot(8,8,32);
    rx_IQ_slice = rx_IQ4(2049:end,4);
    rx_IQ_rs = reshape(rx_IQ_slice,FFTSIZE,length(rx_IQ_slice)/FFTSIZE);
    f = linspace(-20,20,FFTSIZE);
    fft_mag = abs(fftshift(fft(rx_IQ_rs)));
    plot(f,20*log10(mean(fft_mag,2)))
    title('Node4: FFT Magnitude of rx\_IQ_{RFD}')
    xlabel('Frequency (MHz)')
    %linkaxes(ax,'x')
    axis([-20, 20,-20,40])

    drawnow

    if (~RUN_CONTINOUSLY)
       break 
    end

end

wl_basebandCmd(nodes,'RF_ALL','tx_rx_buff_dis');
wl_interfaceCmd(nodes,'RF_ALL','tx_rx_dis');
