function estimatedPosition_LLA = position(estimatedPosition_LLA, estimatedVelocity_NED, timeStepIMU)

estimatedPosition_LLA(3)  = estimatedPosition_LLA(3) - estimatedVelocity_NED(3)* timeStepIMU;

den = 1 - 0.0818191908425^2 .* (sin(estimatedPosition_LLA(1))).^2;

RM = 6378137.0 .* (1-0.0818191908425^2) ./ (den).^(3/2);

estimatedVelocity_NED(1) = estimatedVelocity_NED(1)/(RM + estimatedPosition_LLA(3));

estimatedPosition_LLA(1) = estimatedPosition_LLA(1) +  estimatedVelocity_NED(1)*timeStepIMU;

den = 1 - 0.0818191908425^2 .* (sin(estimatedPosition_LLA(1))).^2;

RN = 6378137.0 ./ sqrt(den);

estimatedVelocity_NED(2) = estimatedVelocity_NED(2)/((RN + estimatedPosition_LLA(3)) * cos (estimatedPosition_LLA(1)));

estimatedPosition_LLA(2) = estimatedPosition_LLA(2) + estimatedVelocity_NED(2)* timeStepIMU;

estimatedPosition_LLA = [estimatedPosition_LLA(1) estimatedPosition_LLA(2) estimatedPosition_LLA(3)];

end
