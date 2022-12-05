function  [F, G] = dynamicModel(northVelocity,eastVelocity,downVelocity,latitude,altitude,specificForce_Nav, DCMbn,RM,RN)
a11 = 0;
a12 = -( (7.292115e-5 * sin(latitude)) + (eastVelocity / (sqrt(RN*RM) + altitude) * tan(latitude)) );
a13 = northVelocity / (sqrt(RN*RM) + altitude);
a21 = (7.292115e-5 * sin(latitude)) + (eastVelocity / (sqrt(RN*RM) + altitude) * tan(latitude));
a22 = 0 ;
a23 = (7.292115e-5 * cos(latitude)) + (eastVelocity / (sqrt(RN*RM) + altitude)) ;
a31 = -northVelocity / (sqrt(RN*RM) + altitude);
a32 = -7.292115e-5 * cos(latitude) - (eastVelocity / (sqrt(RN*RM) + altitude));
a33 = 0;
F11 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = 0;
a12 = 1 / (sqrt(RN*RM) + altitude);
a13 = 0;
a21 = -1 / (sqrt(RN*RM) + altitude);
a22 = 0;
a23 = 0;
a31 = 0;
a32 = -tan(latitude) / (sqrt(RN*RM) + altitude);
a33 = 0;
F12 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = -7.292115e-5 * sin(latitude);
a12 = 0;
a13 = -eastVelocity / ((sqrt(RN*RM) + altitude)^2);
a21 = 0 ;
a22 = 0 ;
a23 = northVelocity / ((sqrt(RN*RM) + altitude)^2);
a31 =  -7.292115e-5 * cos(latitude) - (eastVelocity / (((sqrt(RN*RM) + altitude)) * (cos(latitude))^2));
a32 = 0 ;
a33 = (eastVelocity * tan(latitude)) / ((sqrt(RN*RM) + altitude)^2) ;
F13 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

F21 = formskewsym(specificForce_Nav);

a11 = downVelocity / (sqrt(RN*RM) + altitude);
a12 = -2 * ((7.292115e-5 * sin(latitude)) + ((eastVelocity / (sqrt(RN*RM) + altitude)) * tan(latitude))) ;
a13 = northVelocity / (sqrt(RN*RM) + altitude) ;
a21 = (2 * 7.292115e-5 * sin(latitude)) + ( (eastVelocity / (sqrt(RN*RM) + altitude)) * tan(latitude) );
a22 = (1 / (sqrt(RN*RM) + altitude)) * ((northVelocity * tan(latitude)) + downVelocity) ;
a23 = 2 * 7.292115e-5 * cos(latitude) + (eastVelocity / (sqrt(RN*RM) + altitude));
a31 = (-2 * northVelocity) / (sqrt(RN*RM) + altitude);
a32 = -2 * (7.292115e-5 * cos(latitude) +  (eastVelocity / (sqrt(RN*RM) + altitude))) ;
a33 = 0;
F22 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

e = 0.0818191908425;
res = RN * sqrt( cos(latitude)^2 + (1-e^2)^2 * sin(latitude)^2);
g = gravity(latitude,altitude);
g0 = g(3);

a11 = -eastVelocity * ((2 * 7.292115e-5 * cos(latitude)) + (eastVelocity / ((sqrt(RN*RM) + altitude) * (cos(latitude))^2)));
a12 = 0 ;
a13 = (1 / (sqrt(RN*RM) + altitude)^2) * ( (eastVelocity^2 * tan(latitude)) - (northVelocity * downVelocity) );
a21 = 2 * 7.292115e-5 * ( (northVelocity * cos(latitude)) - (downVelocity * sin(latitude)) ) + ( (northVelocity * eastVelocity) / ((sqrt(RN*RM) + altitude) * (cos(latitude))^2) ) ;
a22 = 0 ;
a23 = -(eastVelocity / (sqrt(RN*RM) + altitude)^2) * (northVelocity * tan(latitude) + downVelocity);
a31 = 2 * 7.292115e-5 * eastVelocity * sin(latitude);
a32 = 0;
a33 = eastVelocity^2 / (RN+altitude)^2 + northVelocity^2 / (RM+altitude)^2 - 2 * g0 / res;
F23 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

F31 = zeros(3);

a11 = 1 / (sqrt(RN*RM) + altitude);
a12 = 0;
a13 = 0;
a21 = 0;
a22 = 1 / ((sqrt(RN*RM) + altitude) * cos(latitude));
a23 = 0;
a31 = 0;
a32 = 0;
a33 = -1;
F32 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

a11 = 0;
a12 = 0;
a13 = -northVelocity / (sqrt(RN*RM) + altitude)^2;
a21 = (eastVelocity * tan(latitude)) / ((sqrt(RN*RM) + altitude) * cos(latitude));
a22 = 0;
a23 = -eastVelocity / ((sqrt(RN*RM) + altitude)^2 * cos(latitude));
a31 = 0;
a32 = 0;
a33 = 0;
F33 = [a11 a12 a13; a21 a22 a23; a31 a32 a33;];

%% Forming Dynamic Model
F = [F11 F12 F13 DCMbn zeros(3)  ;
    F21  F22 F23 zeros(3)     DCMbn  ;
    F31  F32 F33 zeros(3)     zeros(3)      ;
    zeros(3)    zeros(3)   zeros(3)   diag([-0.001 -0.001 -0.001])   zeros(3)      ;
    zeros(3)    zeros(3)   zeros(3)   zeros(3)     diag([-0.001 -0.001 -0.001])    ;
    ];

%% Forming Noise Distribution Matrix
G = [DCMbn zeros(3)     zeros(3)   zeros(3) ;
    zeros(3)      DCMbn zeros(3)   zeros(3) ;
    zeros(3)      zeros(3)     zeros(3)   zeros(3) ;
    zeros(3)      zeros(3)     eye(3) zeros(3) ;
    zeros(3)      zeros(3)     zeros(3)   eye(3) ;
    ];

end
