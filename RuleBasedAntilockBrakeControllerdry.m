% For dry surface

lambda_max=0.155;
lambda_min=0.145;
a1= 0.0005;
a2= 0.0012;
dT=0;

if lambda(i)<lambda_min
    dT= a1*Torque(i);
elseif lambda(i)>lambda_max
    dT= -a2*Torque(i);
end

Torque(i+1)=Torque(i)+dT;