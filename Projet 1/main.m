% Projet 1 : Machine synchrone

% Définition des variables

Wv=70; % rad/s, puslation des tensions
E=12; % V, tension continue d'alimentation
T=0.1; % s, durée de la simulation

I0 = 55; % A, intensité maximale
Pp=4; % paires de pôles
n=Pp*2; % nombre de tours

Ld=1.15e-3; % H, inductance stator de Park directe
Lq=3.31e-3; % H, inductance stator de Park quadratique
phim=200e-3; % mWb, flux des aimants vu par l'ensemble des spires d'une phase
Rs=0.18; % ohm, résistance du stator

J=800e-6; % Inertie du moteur
k=0.3; % coef de frottement visqueux

% Initialisation des variables utiles
We=[0]; % Pulsation électrique initiale
Wm=[0]; % Pulsation mécanique initiale
A=[0]; % Angle électrique initial

phid=[0.2];
phiq=[0];

dt=1e-3; % Pas de temps
t=[0:dt:T]; % Temps

for i = 1:length(t)
    % Tensions à calculer
    Vo1(i)=E*sign(sin(Wv.*t(i)+pi)); 
    Vo2(i)=E*sign(sin(Wv.*t(i)+pi/3)); 
    Vo3(i)=E*sign(sin(Wv.*t(i)-pi/3)); 
    
    % Calcul de Vd et Vq
    [Vd(i),Vq(i)]=park(Vo1(i),Vo2(i),Vo3(i),A(i));
    
    % Calcul des flux direct et quadratique
    phid(i+1)=(Vd(i)+We(i)*phiq(i)-Rs/Ld*(phid(i)-phim))*dt+phid(i);
    phiq(i+1)=(Vq(i)-We(i)*phid(i)-Rs/Lq*phiq(i))*dt+phiq(i);

    % Calcul des courants direct et quadratique
    Id(i)=(phid(i)-phim)/Ld;
    Iq(i)=phiq(i)/Lq;

    % Calcul des courants triphasés par transformée de Park inverse
    [I1(i),I2(i),I3(i)]=park_1(Id(i),Iq(i),A(i));

    % Calcul des couples
    C(i)=Pp*(phid(i)*Iq(i)-phiq(i)*Id(i)); % Couple réel
    Cr(i)=k*We(i); % Couple visqueux

    % Calcul des vitesses angulaires
    dWm(i+1)=(C(i)-Cr(i))/J; % Accélération mécanique
%     dWm(i+1)=C(i)/J; % Suppression du couple résistant
    Wm(i+1)=dWm(i+1)*dt+Wm(i); % Vitesse mécanique
    We(i+1)=Wm(i+1)*Pp; % Vitesse électrique

    % Calcul de l'angle électrique
    A(i+1)=We(i+1)*dt+A(i);
end

% Suppression du dernier terme pour avoir le bon nombre de valeurs
We(length(We))=[];
A(length(A))=[];
phid(length(phid))=[];
phiq(length(phiq))=[];

% Tracer des courbes
% Tensions triphasées de l'onduleur
figure();
title("Tensions triphasées de l'onduleur V_{o1}, V_{o2} et V_{o3}");
hold on
plot(t,Vo1+5);
plot(t,Vo2);
plot(t,Vo3-5);
legend("V_{o1} + 5V","V_{o2}","V_{o3} - 5V");
xlabel("Temps (s)");
ylabel("Tension (V)");
hold off

% Tensions directe et quadratique
figure()
title("Tensions directe et quadratique V_{d} et V_{q}");
hold on
plot(t,Vd);
plot(t,Vq);
legend("V_{d}","V_{q}");
xlabel("Temps (s)");
ylabel("Tension (V)");
hold off

% Flux direct et quadratique
figure()
title("Flux direct et quadratique \phi_{d} et \phi_{q}");
hold on
plot(t,phid);
plot(t,phiq);
legend("\phi_{d}","\phi_{q}");
xlabel("Temps (s)");
ylabel("Flux de champ magnétique (mWb)")
hold off

% Courants direct et quadratique
figure()
title("Courants direct et quadratique I_{d} et I_{q}");
hold on 
plot(t,Id);
plot(t,Iq);
legend("I_{d}","I_{q}");
xlabel("Temps (s)");
ylabel("Intensité (A)");
hold off

% Courants triphasés dans la machine
figure()
title("Courants triphasées de l'onduleur I_{1}, I_{2} et I_{3}");
hold on
plot(t,I1);
plot(t,I2);
plot(t,I3);
legend("I_{1}","I_{2}","I_{3}");
xlabel("Temps (s)");
ylabel("Intensité (A)");
hold off

% Grandeurs mécaniques C, We et A
figure()
title("Grandeurs mécaniques");
hold on
plot(t,C);
plot(t,We/10);
plot(t,5*A);
legend("C (N.m)","\omega_{e}/10 (rad/s)","5A (rad)");
xlabel("Temps (s)");
hold off
