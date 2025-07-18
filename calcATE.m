function [errorResult_ATE, trajectoryNormError] = calcATE(trajectoryEsti, trajectoryTrue, errorType)
% Project:  Patch-based illumination variant DVO
% Function: calcATE
%
% Description:
%   calculate Absolute Trajectory Error (ATE) defined well in TUM benchmark dataset.
%   It is designed to compare the absolute distances btwn the estimated and the ground truth trajectory
%
% Example:
%   OUTPUT :
%   errorResult_ATE: error result of ATE with respect to the 'errorType'
%   trajectoryNormError: norm error of the translational components in trajectory
%
%   INPUT :
%   stateEsti: current estimated 6-dof state w.r.t. ntimeRgb stamps
%   stateTrue: ground truth 6-dof state w.r.t. ntimeRgb stamps
%   errorType : the type of error related to ATE ( RMSE / MEAN / MEDIAN )
%
% NOTE:
%     Absolute Trajectory Error (ATE) is well explained in the below paper
%     "A Benchmark for the Evaluation of RGB-D SLAM Systems" at page 579
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

trajectoryError = cell(1,numImg);
trajectoryNormError = zeros(1,numImg);

%% main part

for i=1:numImg
    % calculate ATE
    estiPart = inv(trajectoryEsti{i});
    truePart = inv(trajectoryTrue{i});
    trajectoryError{i} =   inv( truePart ) * ( estiPart );
    trajectoryNormError(i) = norm( trajectoryError{i}(1:3,4) );
end

%% assigne the error value
if ( strcmp(errorType,'RMSE') )
    
    errorResult_ATE = sqrt( sum( trajectoryNormError.^2 ) / numImg );
    
elseif ( strcmp(errorType,'MEAN') )
    
    errorResult_ATE = mean( trajectoryNormError );
    
elseif ( strcmp(errorType,'MEDIAN') )
    
    errorResult_ATE = median( trajectoryNormError );
    
else
    
    error('errorType error!!! Try again!!');
    
end

end
