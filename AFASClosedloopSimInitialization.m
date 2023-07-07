
%The following constant/variables need to be set to run the AFAS
%closed-loop simulation.

% Under Root Dir: OwnshipACConfig
OwnshipAircraftConfig =0;
%In the DDAS: waypoint
cellInfo = { ... 
  { ... 
    'waypoint', ... 
    '', ... 
    '', ... 
    'Auto', ... 
    '-1', ... 
    '0', {... 
{'x', 1, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'y', 1, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'z', 1, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'t', 1, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 
Simulink.Bus.cellToObject(cellInfo) 
%In the DDAS: waypoint
cellInfo = { ... 
  { ... 
    'position', ... 
    '', ... 
    '', ... 
    'Auto', ... 
    '-1', ... 
    '0', {... 
{'x', 1, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'y', 1, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
{'z', 1, 'double', 'real', 'Sample', 'Fixed', [], [], '', ''}; ...
    } ...
  } ...
}'; 
Simulink.Bus.cellToObject(cellInfo) 
%==========================================================================
% Load setup parameters for FCC Mode Logic and Behavior Logic
%==========================================================================
FCCVerticalMode = 2; % 1 = PIT; 2 = ALT Hold/Select; 3 = VS Hold; 4 = GS; 5 = FLC;
FCCLatDirMode   = 2; % 1 = ROL; 2 = HDG Hold/Select; 3 = LOC; 4 = VOR;        
ydEngaged       = 1;

%==========================================================================
% Load gains for Autothrottle
%==========================================================================
        AT_spd_hold_tas_lp_tau        = 5.0;     % TAS LP filter time constant (proportional path)
        AT_spd_hold_tas_val_lp_tau    = 0.5;     % TAS LP filter time constant (rate deriver path)
        AT_spd_hold_ptch_tau_gain     = 0.15;    % rate derivative filter time constant schedule gain
        AT_spd_hold_rate_damping_gain = 10;      % rate damping gain
        AT_spd_hold_cmpst             = 0.5;
        AT_svo_composite              = 1;
        AT_lead_T2=1;
        AT_lead_T1=1;
        % No structural mode filter
        AT_struct_num = 1;
        AT_struct_den = 1;
%==========================================================================
% Load gains for all the vertical modes
%==========================================================================
        pitch_lead_T2=1;
        pitch_lead_T1=1;

        % PI controller gains
        pitch_svo_integral         =0.6;%0.6;
        pitch_svo_composite        =1;
        
        %PIT range/rate limiter gains
        pitch_att_up_limit         =20;       %pitch atitude up limit,deg
        pitch_att_down_limit       =15;       %pitch atitude down limit,deg
        pitch_cmd_rate_limit       =5;        %pitch comand slew rate limit,deg/sec
        pitch_accel_limit          =0.3;      %normal acceleration limit,g
        % Pitch attitude gain
        pitch_att_gain        =0.7;%0.7;
        
        % Pitch rate gain
        pitch_schedule_type   = 3;
        % 1 - one rate gain for one airspeed
        % Trim condition:F00_TP0001_IAS140Kt_TAS142Kt_M022_Q066_H01000Ft_m15000lb_xcg28_LG_UP.mat
        cas_mps                = 72.0222; %140kt
        tas_mps                = 73.051; %142kt
        pitch_rate_gain        = -1400;%-602.8;
        % Trim condition:F00_TP0025_IAS380Kt_TAS385Kt_M058_Q489_H01000Ft_m15000lb_xcg28_LG_UP.mat
%         pitch_rate_gain        = -415.8;
        % 2 - piecewise linear gain scheduling
        pitch_rate_gain_spd   = [85          100.0       120.00      140.00      160.00      180.00      200.00      220.00      240.00      260.00      275.00  ] * 0.5144444;
        pitch_rate_gain_schd  = [4.24        2.80        2.24        1.68        1.44        1.22        0.97        0.85        0.77        0.72        0.66    ] * 1000;
        % 3 - two-point gain scheduling scheme (linear in dB)
        pitch_rate_gain_IAS1  = 100;             % lower airspeed, knots
        pitch_rate_gain_IAS2  = 380;            % higher airspeed, knots
        pitch_rate_gain_GAIN1 = -56;          % gain for lower airspeed, dB
        pitch_rate_gain_GAIN2 = -52.25;          % gain for higher airspeed, dB
              
        % No structural mode filter
        pitch_struct_num = 1;
        pitch_struct_den = 1;

        %==========================================================================
        % Altitude Hold/Select Gain Design and Analysis 
        %==========================================================================
        alt_hold_hold  = 0.1;                     % altitude hold gain
        alt_hold_vs    = 0.58;                     % vertical speed gain
        alt_hold_av    = 1.5;                     % vertical acceleration gain
        % Continuous-time system equivalence of the ADC H/W elliptic digital filter
%        [A_temp,B_temp,C_temp,D_temp] = ellip(3,1,50,2.935/500*2);
%        vs_ellip_dis=ss(A_temp,B_temp,C_temp,D_temp,0.002);
%        vs_ellip_con=d2c(GDC_vs_ellip_dis,'zoh');
%        alt_ellip_dis=ss(A_temp,B_temp,C_temp,D_temp,0.002);
%        alt_ellip_con=d2c(GDC_alt_ellip_dis,'zoh');
%         av_hp_fltr_tau = 10;                    % vertical acceleration high pass filter time constant
%         av_lp_fltr_tau = 0.4;                   % vertical acceleration low pass filter time constant
        
        %==========================================================================
        % VS Hold Gain Design and Analysis
        %==========================================================================
        av_hp_fltr_tau = 10;                    % vertical acceleration high pass filter time constant
        av_lp_fltr_tau = 0.5;                   % vertical acceleration low pass filter time constant

        vs_hold_vs     = 0.64;                   % vertical speed gain
        vs_hold_av     = 1.1;                    % vertical acceleration gain
        
        %==========================================================================
        % Glide Slope Gain Design and Analysis
        %==========================================================================
        gs_rd_hp_tau      = 10;                  % rate deriver filter time constant
        gs_trk_dev_lp_tau = 1;                   % deviation low pass filter time constant
        gs_trk_ax         = 0;                   % longitudinal acceleration gain
        gs_trk_track      = 0.1;                 % deviation track gain
        gs_trk_dev        = 1.4075;              % deviation rate gain
%         gs_trk_dev        = 1.4075*(1.3/(1.3+gs_trk_track*1.4075/(0.3*1.5)^2));
%         gs_trk_av         =1.3;
        gs_trk_av         = 1.3+gs_trk_track*gs_trk_dev/(0.3*1.5)^2;                 % vertical acceleration gain
        
        %==========================================================================
        % Flight Level Change Gain Design and Analysis
        %==========================================================================
        flc_ias_hold_ias_lp_tau        = 5.0;     % CAS LP filter time constant (proportional path)
        flc_ias_hold_ias_val_lp_tau    = 0.5;    % CAS LP filter time constant (rate deriver path)
        flc_spd_ptch_gain              = 1.1;    % pitch angle gain
        flc_ias_hold_ptch_tau_gain     = 0.15;   % rate deriver filter time constant schedule gain
        flc_ias_hold_rate_damping_gain = 10;      % rate damping gain
        flc_ias_hold_cmpst             = 0.017;

%==========================================================================
% Load gains for the Roll Innerloop
%==========================================================================        
        roll_lead_T2=1;
        roll_lead_T1=1;
        % PI controller gains
        roll_svo_integral         =0.6;%0.15;
        roll_svo_composite        =0.00001;
        
        % Roll attitude gain (rad/s/rad)
        roll_att_gain         = 0.45;
        
        % Roll rate gain
        roll_schedule_type    = 3;
        alt_ft                = 1000; %1000ft
        mach                  = 0.213; %TAS142kt
        % 1 - one rate gain for one airspeed
        % Trim condition:F00_TP0001_IAS140Kt_TAS142Kt_M022_Q066_H01000Ft_m15000lb_xcg28_LG_UP.mat
        roll_rate_gain        = 561.5;
        % Trim condition:F00_TP0025_IAS380Kt_TAS385Kt_M058_Q489_H01000Ft_m15000lb_xcg28_LG_UP.mat
        %roll_rate_gain        = 115;
        % 2 - piecewise linear gain scheduling
        roll_rate_gain_spd    = [85.00       100.00      120.00      140.00      160.00      200.00      220.00      275.00  ] * 0.5144444;
        roll_rate_gain_schd   = [1.30        0.86        0.71        0.63        0.57        0.50        0.46        0.46    ] * 1000;
        % 3 - two-point gain scheduling scheme (linear in dB)
        roll_rate_gain_IAS1   = 100;             % lower airspeed, knots
        roll_rate_gain_IAS2   = 380;            % higher airspeed, knots
        roll_rate_gain_GAIN1  = 57.25;          % gain for lower airspeed, dB
        roll_rate_gain_GAIN2  = 41.25;          % gain for higher airspeed, dB
        
        % Wing rock algorithm gains
        wingrock_schedule_type = 3; %1: |roll error| scheduled 2: |roll att| scheduled 3: No gain scheduling
        wingrock_enable_alt = 20000; %ft
        wingrock_enable_mach=0.6;
        wingrock_lookup_row=[0 2 5 10]*pi/180; %radian
        wingrock_lookup_data=[2 2  1 1];       

        % No structural mode filter
        roll_struct_num       = 1;
        roll_struct_den       = 1;

%==========================================================================
% Load gains for the Roll Outerloop
%==========================================================================
        lat_accel_aug_lp_tau        = 10;           % roll command augmentation Ay low pass filter time constant

        %==========================================================================
        % HDG Gain Design and Analysis
        %==========================================================================
        hdg_select_hdg_rate_track   = 0.05;         % heading rate track gain
        hdg_select_hdg_rate_damping = 1;          % heading rate damping gain
        hdg_select_hdg_rate_capt    = 0.2;          % heading rate capture gain
        d2r = 0.01745329;
        roll_limit = 30*d2r;                        % Roll attitude cmd limit
        
        %==========================================================================
        % LOC Gain Design and Analysis
        %==========================================================================
        loc_rd_hp_tau_track         = 2.5;          % rate deriver filter time constant
        loc_trk_dev_lp_tau          = 0.1;          % deviation low pass filter time constant
        loc_trk_dev_track           = 0.0088;       % deviation track gain
        loc_trk_dev_rate_track      = 0.0096;       % deviation rate gain
        
        %==========================================================================
        % VOR Gain Design and Analysis
        %==========================================================================
        apr_vor_trk_dev_track_close       = 4.50e-4; % deviation track gain
        apr_vor_trk_dev_lp_track_tau      = 3;       % eviation low pass filter time constant
        apr_vor_trk_hdg_rate_track_close  = 0.01;    % heading rate track gain

%==========================================================================
% Load gains for the Yaw Damper
%==========================================================================
% Lateral acceleration gain ( rad/s/(m/s/s) ), lateral acceleration low pass
% filter time constant (second), and yaw rate high pass filter time
% constant (second)
    ltrl_acc_gain       = -0.04;%-0.02;   % lateral acceleration gain rad/s/(m/s/s)
    ltrl_acc_lp_tau     = 0.5;     % lateral acceleration low pass filter time constant, second
    yaw_rate_hp_tau     = 1.8;     % yaw rate high pass filter time constant, second

% Lead filter (T1*s + 1)/(T2*s + 1), T1>T2
    yd_w = 0.1;   yd_lead = 70;     % specify the frequency (rad/sec) where the maximum phase lead (deg) is desired
    
    yd_a                = 2/(1-sin(yd_lead*0.01745329)) - 1;    % the maximum phase lead is 57.3*asin((a-1)/(a+1))(deg)
    yd_lead_T2          = 1/yd_w/sqrt(yd_a);                    % T2 = 1/w/sqrt(a)
    yd_lead_T1          = yd_a * yd_lead_T2;                    % T1 = a * T2
%     yd_lead_T2          = 0;                    % T2 = 1/w/sqrt(a)
%     yd_lead_T1          = 0;                    % T1 = a * T2


% PI controller gains
    yd_svo_integral         =0.7;%0.7;
    yd_svo_composite        =0.000005;


% Yaw rate gain
    yd_schedule_type    = 4;
    alt_m               = 304.8; %1000ft
    % 1 - one rate gain for one airspeed
    yaw_rate_gain       = 15521;
    % 2 - piecewise linear gain scheduling
    yaw_rate_gain_spd   = [90       97      106     111     118     127     136     149     165     187     192     228     264     295     ] * 0.5144444;
    yaw_rate_gain_schd  = [15521    11532   11532   10115   8951.7  7831.7  6829.2  5739.3  4506.7  3317.9  3391.5  2103.9  1404.5  994.3226];
    % 3 - two-point gain scheduling scheme (linear in dB)
    yaw_rate_gain_IAS1  = 100;           % lower airspeed, knots
    yaw_rate_gain_IAS2  = 340;          % higher airspeed, knots
    yaw_rate_gain_GAIN1 = 37.8352;           % gain for lower airspeed, dB
    yaw_rate_gain_GAIN2 = 24.2654;           % gain for higher airspeed, dB
    % 4 - piecewide linear gain scheduling on IAS and ALT
    yaw_rate_gain_piecewise_spd = [100 150 210 260 380] * 0.5144444;
    yaw_rate_gain_piecewise_alt = [1000 10000 20000 30000 50000] * 0.3048;
    % Design gain at 100kts, 20000ft (target gain 8.49) and calculate the scaling factor for other flight conditions
    yaw_rate_gain_piecewise_schd= [7.68	5.98		3.22	2.59		1.78;
                                   8.08	5.98		3.70	2.77		1.88;
                                   8.49	5.98		3.91	2.96		1.95;
                                   8.49	5.98		3.98	3.09		1.95;
                                   8.49	5.98		3.98	3.13		1.95] * 2506.8/8.49/32.1634;   %32.1634:gain reduction/compensation for lead-lag filter 
    
    % No structural mode filter
    yd_struct_num       = 1;
    yd_struct_den       = 1;
    
%==========================================================================
% Load gains for the Servo->Control Surface Gear Ratio
%==========================================================================    
    TGR                 = 0.25; %178.65;   
%==========================================================================
% Load all the parameters for 6DoF Airframe Model
%==========================================================================
        %==========================================================================
        % Aerodynamic Forces and Moments
        %==========================================================================
        aircraft.ReferenceArea   = 23.23;         % unit as m^2;
        aircraft.ReferenceSpan   = 14.63;         % unit as m;
        aircraft.ReferenceLength = 1.5875;        % unit as m;
        state.CenterOfGravity = [0 0 0];
        state.CenterOfPressure = [0 0 0];
        %==========================================================================
        % Stability and Control Derivatives （De Havilland Beaver
        % Aerodynamic Model）
        %==========================================================================
        Cz_q    = -2.9880;
        Cz_dpt  = -0.1563;
        Cz_df_alpha = -1.2610;
        Cz_df = -1.3770;
        Cz_de_beta = -15.9300;
        Cz_de = -0.3980;
        Cz_alpha3 = 3.4420;
        Cz_alpha = -5.5780;
        Cz0 = -0.0550;
        Cy_r = 0.3666;
        Cy_p = -0.1240;
        Cy_dr_alpha = 0.5238;
        Cy_dr = 0.1158;
        Cy_da = -0.0296;
        Cy_beta = -0.7678;
        Cy0 = -0.0022;
        Cx_q = -0.6748;
        Cx_dr = 0.0341;
        Cx_dpt2_alpha = 0.1453;
        Cx_dpt = 0.1161;
        Cx_df_alpha = 1.1060;
        Cx_df = -0.0945;
        Cx_alpha3 = -5.1620;
        Cx_alpha2 = 5.4590;
        Cx_alpha = 0.0029;
        Cx0 = -0.0355;
        Cn_r = -0.1112;
        Cn_q = 0.1595;
        Cn_p = -0.1585;
        Cn_dr = -0.0827;
        Cn_dpt3 = -0.0030;
        Cn_dpt = -0.0790;
        Cn_da = -0.0039;
        Cn_beta3 = 0.1373;
        Cn_beta = 0.0067;
        Cn0 = -0.0031;
        Cm_r = -0.3118;
        Cm_q = -15.5600;
        Cm_dpt = -0.0790;
        Cm_df = 0.4072;
        Cm_de = -1.9210;
        Cm_beta2 = 0.6921;
        Cm_alpha2 = -2.1400;
        Cm_alpha = -0.6028;
        Cm0 = 0.0945;
        Cl_r = 0.1695;
        Cl_p = -0.5045;
        Cl_dr = 0.0069;
        Cl_da_alpha = -0.0827;
        Cl_da = -0.0992;
        Cl_beta = -0.0618;
        Cl_alpha2_dpt = -0.0141;
        Cl0 = 5.910000000000000e-04;
        Aelv = [-10.950988937452216,7.724574326492542,22.099426571460153;0,0,1;7.344618834533876,-7.102811291469513e+02,-18.531238546212496];
        Aail = [-10.596970844957410,-1.898597491077738,-5.489730066569559;0,0,1;1.087684949600015,-4.022216611739937e+02,-23.387098435123647];
        Arud = [-9.213114206863477,4.094492107862127,11.881106877476014;0,0,1;0.671982197983402,-6.825211232462366e+02,-42.322937043127716];
        Belv = [25.156772968805655,0.002514396343387;0,0;5.869417239602464,-0.282965919576974];
        Bail = [27.463014323328070,-0.019588081294309;0,0;3.018680884493773,-4.032396715387772];
        Brud = [24.570927492907362,0.002884917833002;0,0;7.543602387715101,-0.514036448929570];
        Celv = [0,57.295779513082320,0];
        Cail = [0,57.295779513082320,0];
        Crud = [0,57.295779513082320,0];
        %==========================================================================
        % 6DOF (Euler Angles)
        %==========================================================================
        % Initial position in inertial axes [Xe,Ye,Ze]
        state.XN = -8500; 
        state.XE = 0; 
        state.XD = -alt_m;
        % Initial velocity in body axes [U,v,w]:
        state.U = tas_mps; 
        state.V = 0; 
        state.W = 0;
        %Initial Euler orientation [roll, pitch, yaw]:
        state.Phi = 0; 
        state.Theta = 0.1303; 
        state.Psi = 0;
        %Initial body rotation rates [p,q,r]:
        state.P = 0; 
        state.Q = 0; 
        state.R = 0;
        %Initial mass
        state.Mass = 2288.2;
        %Inertia
        state.Inertia.Variables = [5.787969e3  0 1.1764e2; 0 6.92893e3 0; -1.1764e2 0 1.1578329e4];
        %==========================================================================
        % Environment Models
        %==========================================================================    
        %Reference height from Earth's surface to Flat Earth frame [m]
        state.GroundHeight = 45.5;

% It is just for testing
PreviousWaypoint = [118 39 10];