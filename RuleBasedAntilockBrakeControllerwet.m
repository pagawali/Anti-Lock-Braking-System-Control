% For wet surface

lambda_max=0.22;
lambda_min=0.18;
a1= 4e-3;
a2= 9e-3;
dT=0;

if lambda(i)<lambda_min
    dT= a1*Torque(i);
elseif lambda(i)>lambda_max
    dT= -a2*Torque(i);
end

Torque(i+1)=Torque(i)+dT;