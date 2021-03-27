%% STEINER %% 
% STEINER team
% Date: 27/03/21
% File: setup_path.m 
% Issue: 0 
% Validated: 

%% Set up path %%
% This scripts provides the function to generate the needed search paths 

function setup_path()
    %Generate the search paths of the main subfolders of the program
    mainFolder = fileparts(which('setup_path'));
    folderDyn = strcat(mainFolder,'\Dynamics');
    folderEnv = strcat(mainFolder,'\Environment');
    folderFor = strcat(mainFolder,'\Forces');
    
    %Add paths
    addpath(genpath(folderDyn), '-end');
    addpath(genpath(folderEnv), '-end');
    addpath(genpath(folderFor), '-end');
end