function [errorResult_RPE, trajectoryNormError] = calcRPE(trajectoryEsti, trajectoryTrue, timeInterval, errorType)
% Project:  Patch-based illumination variant DVO
% Function: calcRPE
%
% Description:
%   calculate Relative Pose Error (RPE) defined well in TUM benchmark dataset.
%   It is designed to measure the local accuracy of the trajectory over a fixed time interval
%
% Example:
%   OUTPUT :
%   errorResult_RPE: error result of RPE with respect to the 'errorType'
%   trajectoryNormError: norm error of the translational components in trajectory
%
%   INPUT :
%   stateEsti: current estimated 6-dof state w.r.t. ntimeRgb stamps
%   stateTrue: ground truth 6-dof state w.r.t. ntimeRgb stamps
%   timeInterval : a fixed time interval ( 30 if 30Hz -> drift m/s )
%   errorType : the type of error related to RPE ( RMSE / MEAN / MEDIAN )
%
% NOTE:
%     Relative Pose Error (RPE) is well explained in the below paper
%     "A Benchmark for the Evaluation of RGB-D SLAM Systems" at page 578
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2015-02-05 : Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

%% set parameters

% [trajectoryEsti,trajectoryTrue] = preprocessingforRPE_ATE(stateEsti,stateTrue);
%trajectoryEsti : estimated trajectory consists of [{P1}, {P2}, ..... ,{PnumImg}]
%trajectoryTrue : ground truth trajectory consists of [{Q1}, {Q2}, ..... , {QnumImg}]

numImg = size(trajectoryEsti,2);
numRPE = numImg - timeInterval; % the number of RPE

trajectoryError = cell(1,numRPE);
trajectoryNormError = zeros(1,numRPE);


%% main part

for i=1:numRPE
    % truePart
    Qtrue = inv(trajectoryTrue{i});
    QtrueNext = inv(trajectoryTrue{i + timeInterval});
    
    % estiPart
    Pesti = inv(trajectoryEsti{i});
    PestiNext = inv(trajectoryEsti{i + timeInterval});
    
    % calculate RPE
    truePart = inv(Qtrue) * QtrueNext;
    estiPart = inv(Pesti) * PestiNext;
    trajectoryError{i} =   inv( truePart ) * ( estiPart );
    trajectoryNormError(i) = norm( trajectoryError{i}(1:3,4) );
end

%% assigne the error value
if ( strcmp(errorType,'RMSE') )
    
    errorResult_RPE = sqrt( sum( trajectoryNormError.^2 ) / numRPE );
    
elseif ( strcmp(errorType,'MEAN') )
    
    errorResult_RPE = mean( trajectoryNormError );
    
elseif ( strcmp(errorType,'MEDIAN') )
    
    errorResult_RPE = median( trajectoryNormError );
    
else
    
    error('errorType error!!! Try again!!');
    
end

end
