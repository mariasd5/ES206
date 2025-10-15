function [Vd,Vq]=park(V1,V2,V3,theta)
    % Transformée de Park
    Vd=sqrt(2/3)*(cos(theta)*V1 + cos(theta-2*pi/3)*V2 + cos(theta-4*pi/3)*V3);
    Vq=-sqrt(2/3)*(sin(theta)*V1 + sin(theta-2*pi/3)*V2 + sin(theta-4*pi/3)*V3);
end