function [qua, DCMbody2nav, euler] = attitude(wb_corrected, DCMbody2nav, omegaEci2Ecef_Nav_skew, omegaEci2Navi_Nav_skew, timeStepIMU)

%% Gyros output correction for Earth and transport rates
omegaEci2Ecef_Nav = formskewsym_inv(omegaEci2Ecef_Nav_skew);
omegaEci2Navi_Nav = formskewsym_inv(omegaEci2Navi_Nav_skew);
wb_corrected = (wb_corrected - DCMbody2nav' * (omegaEci2Ecef_Nav + omegaEci2Navi_Nav));

% Updating Rotation Matrix (Using High-Precision Method)
A = expm(formskewsym(wb_corrected*timeStepIMU));

DCMbody2nav = DCMbody2nav*A;

c1 = DCMbody2nav(:,1);
c2 = DCMbody2nav(:,2);
c3 = DCMbody2nav(:,3);

c1 = c1 - 0.5 * (c1'*c2) * c2 - 0.5 * (c1'*c3) * c3 ;
c2 = c2 - 0.5 * (c1'*c2) * c1 - 0.5 * (c2'*c3) * c3 ;
c3 = c3 - 0.5 * (c1'*c3) * c1 - 0.5 * (c2'*c3) * c2 ;

c1 = c1 / sqrt(c1'*c1);
c2 = c2 / sqrt(c2'*c2);
c3 = c3 / sqrt(c3'*c3);

DCMbody2nav = [c1 , c2 , c3 ];

% Updating Euler Angles for Plotting
euler(1,1)   =  atan( DCMbody2nav(3,2) ./ DCMbody2nav(3,3) );  
euler(1,2) = -asin( DCMbody2nav(3,1) );                
euler(1,3)    =  atan2( DCMbody2nav(2,1), DCMbody2nav(1,1) );   

% Converting Euler Angles to Quaternions
[~,theta_rad,k] = genAngleAxis(DCMbody2nav);
[q,~] = genQuat('rad',theta_rad,k);
qua = [q(2:4);q(1)];

end
