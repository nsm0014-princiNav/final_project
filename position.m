function estimatedPosition_LLA = position(estimatedPosition_LLA, estimatedVelocity_NED, timeStepIMU)

estimatedPosition_LLA(3)  = estimatedPosition_LLA(3) - estimatedVelocity_NED(3)* timeStepIMU;

[RM, ~] = radius(estimatedPosition_LLA(1));

 estimatedVelocity_NED(1) = estimatedVelocity_NED(1)/(RM + estimatedPosition_LLA(3));

estimatedPosition_LLA(1) = estimatedPosition_LLA(1) +  estimatedVelocity_NED(1)*timeStepIMU;

[~, RN] = radius(estimatedPosition_LLA(1));

estimatedVelocity_NED(2) = estimatedVelocity_NED(2)/((RN + estimatedPosition_LLA(3)) * cos (estimatedPosition_LLA(1)));

estimatedPosition_LLA(2) = estimatedPosition_LLA(2) + estimatedVelocity_NED(2)* timeStepIMU;

estimatedPosition_LLA = [estimatedPosition_LLA(1) estimatedPosition_LLA(2) estimatedPosition_LLA(3)];

end
