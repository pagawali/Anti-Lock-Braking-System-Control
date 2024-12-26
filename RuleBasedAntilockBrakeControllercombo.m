% For both surface

lambda_max=0.26;
lambda_min=0.1;
a1= 0.05;
a2= 0.08;
dT=0;

if lambda(i)<lambda_min
    dT= a1*Torque(i);
elseif lambda(i)>lambda_max
    dT= -a2*Torque(i);
end

Torque(i+1)=Torque(i)+dT;