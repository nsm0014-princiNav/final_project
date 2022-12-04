function  kf = kalman(kf, dt)
% kalman: Kalman filter algorithm.
%
% INPUT
%   kf, data structure with at least the following fields:
%       xp: nx1 a posteriori state vector.
%        z: rx1 measurement vector.
%        F: nxn state transition matrix.
%        H: rxn observation matrix.
%        Q: qxq process noise covariance matrix.
%        R: rxr observation noise covariance matrix.
%       Pp: nxn a posteriori error covariance matrix.
%        G: nxq control-input matrix.      
%   	dt: sampling interval. 
%
% OUTPUT
%    kf, the following fields are updated:
%       xi: nx1 a priori state vector.
%       xp: nx1 a posteriori state vector.
%		 v: rx1 innovation vector. 
%        A: nxn state transition matrix.
%        K: nxr Kalman gain matrix.
%       Qd: nxn discrete process noise covariance matrix.
%       Pi: nxn a priori error covariance.
%       Pp: nxn a posteriori error covariance.  
%        S: rxr innovation (not residual) covariance matrix.
%
%   Copyright (C) 2014, Rodrigo Gonzalez, all rights reserved.
%
%   This file is part of NaveGo, an open-source MATLAB toolbox for
%   simulation of integrated navigation systems.
%
%   NaveGo is free software: you can redistribute it and/or modify
%   it under the terms of the GNU Lesser General Public License (LGPL)
%   version 3 as published by the Free Software Foundation.
%
%   This program is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied warranty of
%   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%   GNU Lesser General Public License for more details.
%
%   You should have received a copy of the GNU Lesser General Public
%   License along with this program. If not, see
%   <http://www.gnu.org/licenses/>.
%
% Reference:
%
%   Simo Särkkä (2013). Bayesian Filtering and Smoothing. Cambridge 
%     University Press.  
%
% Version: 007
% Date:    2019/04/19
% Author:  Rodrigo Gonzalez <rodralez@frm.utn.edu.ar>
% URL:     https://github.com/rodralez/navego

%% PREDICTION STEP

kf = kf_prediction(kf, dt);

%% UPDATE STEP

kf = kf_update(kf);

end
