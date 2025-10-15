function [V1,V2,V3] = park_1(Vd,Vq,theta)
    % Transformée de Park inverse
    V1=sqrt(2/3)*(cos(theta)*Vd-sin(theta)*Vq);
    V2=sqrt(2/3)*(cos(theta-2*pi/3)*Vd-sin(theta-2*pi/3)*Vq);
    V3=sqrt(2/3)*(cos(theta-4*pi/3)*Vd-sin(theta-4*pi/3)*Vq);
end