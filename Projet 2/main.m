% Projet 2 : Modulation de largeur d'impulsion

% Définition des variables

Wv=220; % rad/s, puslation des tensions
E=50; % V, tension continue d'alimentation
T=0.02; % s, durée de la simulation

I0 = 55; % A, intensité maximale
Pp=4; % paires de pôles
n=Pp*2; % nombre de tours

Ld=1.15e-3; % H, inductance stator de Park directe
Lq=3.31e-3; % H, inductance stator de Park quadratique
phim=200e-3; % mWb, flux des aimants vu par l'ensemble des spires d'une phase
Rs=0.18; % ohm, résistance du stator

J=800e-6; % Inertie du moteur
k=0.2; % coef de frottement visqueux

% Initialisation des variables utiles
We=[0]; % Pulsation électrique initiale
Wm=[0]; % Pulsation mécanique initiale
A=[0]; % Angle électrique initial

phid=[0.2];
phiq=[0];

tp=1e-4; % Période de découpage
tint=tp/50; % Temps interne
t0=[0:tp:T]; % Liste des temps - grand pas
t=[0:tint:T]; % Liste des temps - petit pas

for i = 1:length(t0)
    % Tensions d'entrée de l'onduleur
    RVo1(i)=E*sin(Wv*t0(i)+pi);
    RVo2(i)=E*sin(Wv*t0(i)+pi/3);
    RVo3(i)=E*sin(Wv*t0(i)-pi/3);

    % Temps tri et tfi, i=1,2,3
    tr1(i)=tp*(1-RVo1(i)/E)/4;
    tf1(i)=tp*(3+RVo1(i)/E)/4;

    tr2(i)=tp*(1-RVo2(i)/E)/4;
    tf2(i)=tp*(3+RVo2(i)/E)/4;

    tr3(i)=tp*(1-RVo3(i)/E)/4;
    tf3(i)=tp*(3+RVo3(i)/E)/4;
end

% Calcul de Vo1
l=1;
for i = 1:length(t)
    if t(i)<=tr1(l)+(l-1)*tp
        Vo1(i) = -E;
    elseif t(i)<=tf1(l)+(l-1)*tp
        Vo1(i) = E;
    elseif t(i)<=tf1(l)+tr1(l)+(l-1)*tp
        Vo1(i) = -E;
    else 
        Vo1(i)=-E;
        l=l+1;
    end
end

% Calcul de Vo2
l=1;
for i = 1:length(t)
    if t(i)<=tr2(l)+(l-1)*tp
        Vo2(i) = -E;
    elseif t(i)<=tf2(l)+(l-1)*tp
        Vo2(i) = E;
    elseif t(i)<=tf2(l)+tr2(l)+(l-1)*tp
        Vo2(i) = -E;
    else 
        Vo2(i)=-E;
        l=l+1;
    end
end

% Calcul de Vo3
l=1;
for i = 1:length(t)
    if t(i)<=tr3(l)+(l-1)*tp
        Vo3(i) = -E;
    elseif t(i)<=tf3(l)+(l-1)*tp
        Vo3(i) = E;
    elseif t(i)<=tf3(l)+tr3(l)+(l-1)*tp
        Vo3(i) = -E;
    else 
        Vo3(i)=-E;
        l=l+1;
    end
end


for i=1:length(t)
    % Calcul de Vd et Vq
    [Vd(i),Vq(i)]=park(Vo1(i),Vo2(i),Vo3(i),A(i));
    
    % Calcul des flux direct et quadratique
    phid(i+1)=(Vd(i)+We(i)*phiq(i)-Rs/Ld*(phid(i)-phim))*tint+phid(i);
    phiq(i+1)=(Vq(i)-We(i)*phid(i)-Rs/Lq*phiq(i))*tint+phiq(i);

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
    Wm(i+1)=dWm(i+1)*tint+Wm(i); % Vitesse mécanique
    We(i+1)=Wm(i+1)*Pp; % Vitesse électrique

    % Calcul de l'angle électrique
    A(i+1)=We(i+1)*tint+A(i);
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
plot(t(1:100),Vo1(1:100)+5);
plot(t(1:100),Vo2(1:100));
plot(t(1:100),Vo3(1:100)-5);
plot(t0(1:3),tr1(1:3)*1e6);
plot(t0(1:3),tf1(1:3)*1e6);
legend("V_{o1} + 5 (V)","V_{o2} (V)","V_{o3} - 5 (V)","t_{r1}*10^{6} (s)","t_{f1}*10^{6} (s)");
xlabel("Temps (s)");
hold off

% Tensions directe et quadratique
figure()
title("Tensions directe et quadratique V_{d} et V_{q}");
hold on
plot(t(1:100),Vd(1:100));
plot(t(1:100),Vq(1:100));
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

% Tensions d'entrée RVo1,2,3
figure()
title("Tensions d'entrée RV_{o1}, RV_{o2} et RV_{o3}");
hold on
plot(t0,RVo1);
plot(t0,RVo2);
plot(t0,RVo3);
legend("RV_{o1}","RV_{o2}","RV_{o3}");
xlabel("Temps (s)");
ylabel("Tension (V)");
hold off