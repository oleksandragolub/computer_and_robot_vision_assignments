% Computer e robot vision
% Versione dell'esercizio: 5 del 01.jan.2019
% 
% Compilare i seguenti campi:
% Nome: 
% Cognome: 
% Matricola: 
% 
% Note:
%  1) utilizzare il Sistema Internazionale per ogni grandezza dotata di dimensioni,
%  2) per la lunghezza focale usare i pixel,
%  3) utilizzare in modo intensivo i commenti, 
%  4) le sezioni su cui lavorare sono identificate con dei caratteri underscore ("_"), come qui sotto
%     ____________________________________________________________
%  5) supponiamo che il robot sia un corpo rigido piano che si muove nel piano 2D.
%  6) il vettore di stato è composto dai gradi di libertà  del robot nel
%     piano (x,y,theta), dove theta è l'angolo di rotazione del sistema
%     di riferimento solidale con il robot rispetto al sistema di
%     riferimento del mondo




function EKF

    % Close all windows and reset workspace
    clear all;
    close all;

    % Robot & Environment configuration
    filename = 'simulated_data.txt';    % better not to edit the filename for the simulated data
    baseline = ___;                     % distance between the contact points of the robot wheels [m]
    focal = ____;                       % focal length of the camera [pixel]
    camera_height = __;                 % how far is the camera w.r.t. the world plane [m]
    pTpDistance = _;                    % distance between the two observed points on the robot [m]

    % Read & parse data from the input file
    DATA=read_input_file(filename);    
    odometry  = [DATA(:,3),DATA(:,4)];    
    camera_readings = [DATA(:,11),DATA(:,12),DATA(:,13),DATA(:,14)];

    steps = size(odometry,1); 
    kalmanstate = zeros(steps,3);       % memory for the 3 state variables, for each timestamp
    kalmanstatecov = zeros(3,3,steps);  % memory for the covariance of the 3 state variables, for each timestamp
    prediction = zeros(steps,3);        % memory for the 3 predicted state variables, for each timestamp
    predictedcov = zeros(3,3,steps);    % memory for the covariance of the 3 predicted state variables, for each timestamp

    % Initializations 
    kalmanstate(1,:) = _________;       % initial state estimate
    kalmanstatecov(:,:,1) = _________;  % covariance of the initial state estimate, a (3,3) matrix; could it be set to diagonal? insert your reasoning in a comment

    % configuration of uncertainties
    R(:,:) = _________;     % covariance of the noise acting on the state, a (3,3) matrix; could it be set to diagonal? insert your reasoning in a comment
    Q(:,:) = _________;     % covariance of the noise acting on the measure, a (4,4) matrix; could it be set to block diagonal (2,2)? insert your reasoning in a comment

    % Compute symbolic jacobians once and for all! If you do not already know how to use symbolic calculus inside matlab, Please take ten minutes to learn it.
    % Create the symbolic matrices representing the G and H matrices.
    H = calculateSymbolicH; % Use as an example for G, and remind that its analytical form depends on whether the robot is going straight or not
    G = _________;

    % And here we go: batch-simulate in this loop the evolution of the system as well as your KF estimate of its state
    for i=1:steps-1
        % Print the current status of the system in the command window
        fprintf('Filtering step %d/%d; Current uncertainty (x,y,ang): %0.6f %0.6f %0.6f\n',i,steps, sigma(1,1,i), sigma(2,2,i),sigma(3,3,i)); 
        [kalmanstate(i+1,:), kalmanstatecov(:,:,i+1), prediction(i+1,:), predictedcov(:,:,i+1)] = execute_kalman_step(______________);
    end

    % Do not edit the following lines!
    %Plot the results
    figure(3);
    hold on;
        plot(kalmanstates(:,1), kalmanstates(:,2),'g+');      
        plot(predictions(:,1), predictions(:,2),'c.');

        % plot where the robot points would be on the ground plane for a given measurement (if used as a measurement, this expression would be an inverse sensor model)
        plot(camera_readings(:,1)*camera_height/focal, -camera_readings(:,2)*camera_height/focal,'m*');

        % plot of state uncertainty
        for i=1:1:steps
            plot_gaussian_ellipsoid(kalmanstates(i,1:2),sigma(1:2,1:2,i),3.0); 
        end

        axis equal
    hold off; 

end




function simulated_data = read_input_file(filename) % Do not edit this function
simulated_data = dlmread(filename,';'); 
end




function [filtered_state, filtered_sigma, predicted_state, predicted_sigma]= execute_kalman_step(current_state,current_state_sigma,odometry,camera_readings,baseline,R,Q,focal,camera_height,pTpDistance,G,H)

    % memory for the expected camera readings
    expected_camera_readings = zeros(1,4);

    % Evaluate the symbolic matrix with the values by using the eval function
    % Please note this is a *terrible* habit.
    % DO NOT EVER USE EVAL INSIDE A MATLAB LOOP, except for this excercise, whose execution time is not of concern

    syms x y a baseline_symb sdx ssx        
    G = eval(subs(G,[x,y,a,baseline_symb,sdx,ssx],[______________________));

    syms symb_focal symb_pTpDistance symb_camera_height symb_x symb_y symb_alpha
    H = eval(subs(H,[symb_focal,symb_pTpDistance,symb_camera_height,symb_x,symb_y,symb_alpha],[______________________));

    % Generate the expected new pose, based on the previous pose and the controls, i.e., the dead reckoning data
    predicted_state(i+1,:)    = ________________________;
    predicted_sigma(:,:,i+1) = ________________________;

    % then integrate the expected new pose with the measurements
    K = ________________________;
    expected_camera_readings(1,1) = _________________;
    expected_camera_readings(1,2) = _________________;
    expected_camera_readings(1,3) = _________________;
    expected_camera_readings(1,4) = _________________;
    filtered_state = ________________________;
    filtered_sigma = ________________________;
end




function H=calculateSymbolicH
    % *** H MATRIX STEP, CALCULATION OF DERIVATIVES aka JACOBIANS ***
    % Please write here the measurement equation(s)
    syms symb_focal symb_x symb_y symb_theta symb_pTpDistance symb_camera_height
    u1 = _________________
    v1 = _________________
    u2 = _________________
    v2 = _________________
    % derivatives in Matlab can be calculted in this compact way. 
    % This is the same of (you can check it in the command window):
    % syms x    --- declare x as symbolic variable
    % diff(x)  
    H = jacobian([u1;v1;u2;v2],[symb_x,symb_y,symb_theta]);
end