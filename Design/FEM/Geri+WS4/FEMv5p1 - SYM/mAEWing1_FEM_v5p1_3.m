% mAEWing1 FEM v5.1

% xx_2. the rigid connection between the accel and beam grid may have
% different spanwise location. Previous Cmatrix assumes they have same
% spanwise location. The present version considers different spanwise
% locations in both beam grid and accel locations, using cross-product to
% obtain the Cmatrix.

% xx_3: update a typo in Cmatrix

% clear all;
close all;

load effective_nodecord.txt

figure(1);

% beam effective node coordinates

cord_X = effective_nodecord(:,2);
cord_Y = effective_nodecord(:,3);
cord_Z = effective_nodecord(:,4);
hold on;
plot3(cord_X ,cord_Y ,cord_Z,'ro');
axis image;view(59,42);

% load K/M
load FEM_KM.mat


[modeshape,eigvalue] = eigs(FEM1.K,FEM1.M,20,'sm'); % first 20 smallest magnitude value
cycle_freq = sqrt(diag(eigvalue)) % rad/s
freq=sqrt(diag(eigvalue))/2/pi; % Hz
sort(real(freq))

% indices for each degree of freedom in the mode shape vector
dof_u_index = 1:6:size(effective_nodecord,1)*6-5;
dof_v_index = 2:6:size(effective_nodecord,1)*6-4;
dof_w_index = 3:6:size(effective_nodecord,1)*6-3;
dof_beta_x_index = 4:6:size(effective_nodecord,1)*6-2;
dof_beta_y_index = 5:6:size(effective_nodecord,1)*6-1;
dof_beta_z_index = 6:6:size(effective_nodecord,1)*6;

%% ===================================
%% ======= USER INPUT ======
%% ===================================
modeNo = 7; % choose which mode to be plotted
scalefactor = 40; % scale mode shape magnitude in plot

%% ===================================
%% ===================================

% mode shape for different DOFs in the mode shape vector
modeshape_u = modeshape(dof_u_index,modeNo);
modeshape_v = modeshape(dof_v_index,modeNo);
modeshape_w = modeshape(dof_w_index,modeNo);
modeshape_betax = modeshape(dof_beta_x_index,modeNo);
modeshape_betay = modeshape(dof_beta_y_index,modeNo);
modeshape_betaz = modeshape(dof_beta_z_index,modeNo);




hold on;
legendname ={'Undeform'};
plot3(cord_X+modeshape_u*scalefactor,cord_Y,cord_Z,'ko','LineWidth',3); 
legendname  = [legendname,{'Trans. u'}];

plot3(cord_X,cord_Y+modeshape_v*scalefactor,cord_Z,'go','LineWidth',3); 
legendname  = [legendname,{'Trans. v'}];

plot3(cord_X,cord_Y,cord_Z+modeshape_w*scalefactor,'bo','LineWidth',3); 
legendname  = [legendname,{'Trans. w'}];


legend1 = legend(legendname);
set(legend1,'FontSize',15,'Location','Northeast');
%set(gcf,'color','w')

%% ======= accels locations

accel_positions =[
    1              3.480   -17.870 0.0
    2              8.110   -16.000 0.0
    3              7.220   -27.140 0.0
    4              11.86   -25.270 0.0
    5              10.97   -36.420 0.0
    6              15.61   -34.540 0.0
    7              14.72   -45.690 0.0
    8              19.35   -43.810 0.0
    9              18.46   -54.960 0.0
    10             23.10   -53.090 0.0
    11             3.480    17.870 0.0
    12             8.110    16.000 0.0
    13             7.220    27.140 0.0
    14             11.86    25.270 0.0
    15             10.97    36.420 0.0
    16             15.61    34.540 0.0
    17             14.72    45.690 0.0
    18             19.35    43.810 0.0
    19             18.46    54.960 0.0
    20             23.10    53.090 0.0];



% [beam grid id  accel grid id]
BEAM_ACCEL_nodes = ...
    [ 101 1
    101 2
    102 3
    102 4
    103 5
    103 6
    104 7
    104 8
    105 9
    105 10
    %
    201 11
    201 12
    202 13
    202 14
    203 15
    203 16
    204 17
    204 18
    205 19
    205 20
    %
    ];



figure(2);
plot3(cord_X ,cord_Y ,cord_Z,'ro');
axis image;view(59,42);
hold on;
plot3(accel_positions(:,2),accel_positions(:,3),accel_positions(:,4),'go');
axis image;view(59,42);

Naccel = size(BEAM_ACCEL_nodes,1);
Nnode = size(modeshape,1);
Caccel = zeros(Naccel,Nnode);
for num_accel_con = 1:Naccel
    % find out the coordinates for each grid with its label
    % beam grid coordinate
    [cord_beam_grid,index_in_beam]= label2cord(BEAM_ACCEL_nodes(num_accel_con,1),effective_nodecord);
    
    % accel grid coordinate
    [cord_accel_grid,index_in_accels ]= label2cord(BEAM_ACCEL_nodes(num_accel_con,2),accel_positions);
    
    hold on;
    plot3([cord_beam_grid(2) cord_accel_grid(2) ],...
        [cord_beam_grid(3) cord_accel_grid(3) ],...
        [cord_beam_grid(4) cord_accel_grid(4) ],'r-');
    view(90,90)
    
    text(cord_accel_grid(2),cord_accel_grid(3),cord_accel_grid(4),...
        num2str(cord_accel_grid(1)),'FontSize',12);
    
    
    %% relationship between the accel nodes and the beam nodes    
    dependent_pnt_cord  = cord_accel_grid;
    independent_pnt_cord = cord_beam_grid;
    
   % relationship for obtaining the STINGER motion based on the beam grid
    % motion for each STINGER, {d_STINGER} = [C]*{d_beam}
    D_X = dependent_pnt_cord(2)-independent_pnt_cord(2);
    D_Y = dependent_pnt_cord(3)-independent_pnt_cord(3);
    D_Z = dependent_pnt_cord(4)-independent_pnt_cord(4);
    

    
% %     Cmatrix = [
% %         1 0 0 0  (dependent_pnt_cord(4)-independent_pnt_cord(4)) 0
% %         0 1 0   -(dependent_pnt_cord(4)-independent_pnt_cord(4)) 0 (dependent_pnt_cord(2)-independent_pnt_cord(2))
% %         0 0 1 0 -(dependent_pnt_cord(2)-independent_pnt_cord(2)) 0
% %         0 0 0 1 0 0
% %         0 0 0 0 1 0
% %         0 0 0 0 0 1 ];
    

    Cmatrix = [
        1 0 0      0    D_Z  -D_Y
        0 1 0    -D_Z   0    D_X
        0 0 1     D_Y  -D_X   0
        0 0 0 1 0 0
        0 0 0 0 1 0
        0 0 0 0 0 1 ];
    
    % mode shape for accels
    % modeNo - mode of interest.
    
    beam_grid_modeshape = [modeshape(dof_u_index(index_in_beam),modeNo)
        modeshape(dof_v_index(index_in_beam),modeNo)
        modeshape(dof_w_index(index_in_beam),modeNo)
        modeshape(dof_beta_x_index(index_in_beam),modeNo)
        modeshape(dof_beta_y_index(index_in_beam),modeNo)
        modeshape(dof_beta_z_index(index_in_beam),modeNo)];
    
    accel_grid_modeshape(:,num_accel_con) = Cmatrix*beam_grid_modeshape;
    
    % Store row of accelerometer output matrix
    idx = [dof_u_index(index_in_beam); ...
        dof_v_index(index_in_beam); ...
        dof_w_index(index_in_beam); ...
        dof_beta_x_index(index_in_beam); ...
        dof_beta_y_index(index_in_beam); ...
        dof_beta_z_index(index_in_beam)];
    
    Caccel(num_accel_con,idx) = Cmatrix(3,:);
end

hold on;
plot3(cord_X,cord_Y,cord_Z+modeshape_w*scalefactor,'bo','LineWidth',3);
hold on;
% plot3(accel_positions(:,2),accel_positions(:,3),...
%     accel_positions(:,4)+accel_grid_modeshape(3,:)'*scalefactor,...
%     'go','LineWidth',3);
plot3(accel_positions(:,2),accel_positions(:,3),...
    accel_positions(:,4)+Caccel*modeshape(:,modeNo)*scalefactor,...
    'go','LineWidth',3);
axis image;view(59,42);


%% ==== Singer location =====


stinger_positions =[
    21              24.3750-15.55   -4.2500 0.0
    22              30.3750-15.55 0 0
];



% [beam grid id  stinger grid id]
BEAM_STINGER_nodes = ...
    [  101001 21
    3006 22
    %
    ];

Nstinger = size(BEAM_STINGER_nodes,1);
Nnode = size(modeshape,1);

% Here Bstinger matrix order is different from Cstinger, transferring force from
% stinger node to that for the structure beam node
Bstinger = zeros(Nnode,Nstinger);

%
% singer only provides input force along z-axis based on shaker info
% for example, u = [0,0,1,0,0,0]^T
% Assume
H = [0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 1 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0
    0 0 0 0 0 0];

for num_stinger_con = 1:Nstinger
    % find out the coordinates for each grid with its label
    % beam grid coordinate
    [cord_beam_grid,index_in_beam]= label2cord(BEAM_STINGER_nodes(num_stinger_con,1),effective_nodecord);
    
    % STINGER grid coordinate
    [cord_STINGER_grid,index_in_STINGERs ]= label2cord(BEAM_STINGER_nodes(num_stinger_con,2),stinger_positions);
    
    hold on;
    plot3([cord_beam_grid(2) cord_STINGER_grid(2) ],...
        [cord_beam_grid(3) cord_STINGER_grid(3) ],...
        [cord_beam_grid(4) cord_STINGER_grid(4) ],'r-');
    view(90,90)
    
    text(cord_STINGER_grid(2),cord_STINGER_grid(3),cord_STINGER_grid(4),...
        num2str(cord_STINGER_grid(1)),'FontSize',12);
    
    
    %% relationship between the STINGER nodes and the beam nodes
    dependent_pnt_cord  = cord_STINGER_grid;
    independent_pnt_cord = cord_beam_grid;
    
    % relationship for obtaining the STINGER motion based on the beam grid
    % motion for each STINGER, {d_STINGER} = [C]*{d_beam}
    D_X = dependent_pnt_cord(2)-independent_pnt_cord(2);
    D_Y = dependent_pnt_cord(3)-independent_pnt_cord(3);
    D_Z = dependent_pnt_cord(4)-independent_pnt_cord(4);
    

    
% %     Cmatrix = [
% %         1 0 0    0  (dependent_pnt_cord(4)-independent_pnt_cord(4)) 0
% %         0 1 0   -(dependent_pnt_cord(4)-independent_pnt_cord(4)) 0 (dependent_pnt_cord(2)-independent_pnt_cord(2))
% %         0 0 1      0 -(dependent_pnt_cord(2)-independent_pnt_cord(2)) 0
% %         0 0 0 1 0 0
% %         0 0 0 0 1 0
% %         0 0 0 0 0 1 ];
    

    Cmatrix = [
        1 0 0      0    D_Z  -D_Y
        0 1 0    -D_Z   0    D_X
        0 0 1     D_Y  -D_X   0
        0 0 0 1 0 0
        0 0 0 0 1 0
        0 0 0 0 0 1 ];
    
    
    % mode shape for STINGERs
    % modeNo - mode of interest.
    
    beam_grid_modeshape = [modeshape(dof_u_index(index_in_beam),modeNo)
        modeshape(dof_v_index(index_in_beam),modeNo)
        modeshape(dof_w_index(index_in_beam),modeNo)
        modeshape(dof_beta_x_index(index_in_beam),modeNo)
        modeshape(dof_beta_y_index(index_in_beam),modeNo)
        modeshape(dof_beta_z_index(index_in_beam),modeNo)];
    
    %     STINGER_grid_modeshape(:,num_stinger_con) = Cmatrix*beam_grid_modeshape;
    
    % Store row of STINGERerometer output matrix
    idx = [dof_u_index(index_in_beam); ...
        dof_v_index(index_in_beam); ...
        dof_w_index(index_in_beam); ...
        dof_beta_x_index(index_in_beam); ...
        dof_beta_y_index(index_in_beam); ...
        dof_beta_z_index(index_in_beam)];
    
    
    B_temp = Cmatrix'*H;
    
    
    Bstinger(idx,num_stinger_con) =  B_temp(:,3); % along z-axis
    

end