% Projet 3 : Commande en tension progressive et surveillance

% Définition des variables

Wv=220; % rad/s, puslation des tensions
E=400; % V, tension continue d'alimentation
T=0.02; % s, durée de la simulation

I0 = 55; % A, intensité maximale
Pp=4; % paires de pôles
n=Pp*2; % nombre de tours

Ld=1.15e-3; % H, inductance stator de Park directe
Lq=3.31e-3; % H, inductance stator de Park quadratique
phim=200e-3; % mWb, flux des aimants vu par l'ensemble des spires d'une phase
Rs=0.18; % ohm, résistance du stator

J=800e-6; % Inertie du moteur
k=0.1; % coef de frottement visqueux

% Initialisation des variables utiles
We=[0]; % Pulsation électrique initiale
Wm=[0]; % Pulsation mécanique initiale
A=[0]; % Angle électrique initial
MA=[0]; % Echantillon de l'angle initial

phid=[0.2];
phiq=[0];

ech=0; % variable utile à l'échantillonnage

tp=1e-4; % Période de découpage
tint=tp/50; % Temps interne
t0=[0:tp:T]; % Liste des temps - grand pas
t=[0:tint:T]; % Liste des temps - petit pas
j=0;

for i=1:length(t)
    % Tensions d'entrée RVd et Rvq
    RVd(i)=-5000*t(i);
    RVq(i)=5000*t(i);

    % Calcul de RV1,2,3 par transformée de Park inverse avec l'angle
    % échantillonné
    [RV1(i),RV2(i),RV3(i)]=park_1(RVd(i),RVq(i),MA(i));

    % Calcul de RVn
    max_R=max([RV1(i),RV2(i),RV3(i)]);
    min_R=min([RV1(i),RV2(i),RV3(i)]);
    RVn(i)=-(max_R+min_R)/2;
%     RVn(i)=0;
    
    % Calcul des tensions d'entrée de l'onduleur
    RVo1(i)=RV1(i)+RVn(i);
    RVo2(i)=RV2(i)+RVn(i);
    RVo3(i)=RV3(i)+RVn(i);    
    
    % Calcul des tri et tfi seulement pour des multiples de tp
    if mod(t(i),tp)==0
        j=j+1;
        % Temps tri et tfi, i=1,2,3
        tr1(j)=tp*(1-RVo1(i)/E)/4;
        tf1(j)=tp*(3+RVo1(i)/E)/4;
        
        tr2(j)=tp*(1-RVo2(i)/E)/4;
        tf2(j)=tp*(3+RVo2(i)/E)/4;
        
        tr3(j)=tp*(1-RVo3(i)/E)/4;
        tf3(j)=tp*(3+RVo3(i)/E)/4;
    end

    
    % Calcul de Vo1
    if mod(t(i),tp)<tr1(j)
        Vo1(i)=-E;
    elseif mod(t(i),tp)<tf1(j)
        Vo1(i)=E;
    else
        Vo1(i)=-E;
    end
    
    % Calcul de Vo2
    if mod(t(i),tp)<tr2(j)
        Vo2(i)=-E;
    elseif mod(t(i),tp)<tf2(j)
        Vo2(i)=E;
    else
        Vo2(i)=-E;
    end

    % Calcul de Vo3
    if mod(t(i),tp)<tr3(j)
        Vo3(i)=-E;
    elseif mod(t(i),tp)<tf3(j)
        Vo3(i)=E;
    else
        Vo3(i)=-E;
    end
    

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

    % Echantillonnages
    if ech==0 
        % Echantillonnage des valeurs tous les tp
        MI1(i)=I1(i);
        MI2(i)=I2(i);
        MI3(i)=I3(i);
        MA(i+1)=A(i);
    else 
        % Blocage pour les autres tours de boucle
        MI1(i)=MI1(i-1);
        MI2(i)=MI2(i-1);
        MI3(i)=MI3(i-1);
        MA(i+1)=MA(i);
    end
    ech=ech+1;
    ech=mod(ech,100);
end

% Suppression du dernier terme pour avoir le bon nombre de valeurs
We(length(We))=[];
A(length(A))=[];
phid(length(phid))=[];
phiq(length(phiq))=[];
MA(length(MA))=[];


% Tracé des courbes
% Tensions triphasées de l'onduleur
figure();
title("Tensions triphasées de l'onduleur V_{o1}, V_{o2} et V_{o3}");
hold on
plot(t(1:100),Vo1(1:100)+100);
plot(t(1:100),Vo2(1:100));
plot(t(1:100),Vo3(1:100)-100);
plot(t(1:100),tr1(1:100)*1e6);
plot(t(1:100),tf1(1:100)*1e6);
legend("V_{o1} + 100 (V)","V_{o2} (V)","V_{o3} - 100 (V)","t_{r1}*10^{6} (s)","t_{f1}*10^{6} (s)");
xlabel("Temps (s)");
hold off

% Tensions directe et quadratique
figure()
title("Tensions de référence RV_{d}, RV_{q} et RV_{n}");
hold on
plot(t,RVd);
plot(t,RVq);
plot(t,-RVn);
legend("RV_{d}","RV_{q}","- RV_{n}");
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
title("Courants I_{1}, I_{2} I_{3}, MI_{1}, MI_{2} et MI_{3}");
hold on
plot(t,I1);
plot(t,I2);
plot(t,I3);
plot(t,MI1);
plot(t,MI2);
plot(t,MI3);
legend("I_{1}","I_{2}","I_{3}","MI_{1}","MI_{2}","MI_{3}");
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
title("Tensions RV_{o1}, RV_{o2} et RV_{o3}");
hold on
plot(t,RVo1);
plot(t,RVo2);
plot(t,RVo3);
legend("RV_{o1}","RV_{o2}","RV_{o3}");
xlabel("Temps (s)");
ylabel("Tension (V)");
hold off

% Tensions d'entrée RV1,2,3
figure()
title("Tensions RV_{1}, RV_{2}, RV_{3} et RV_{n}");
hold on
plot(t,RV1);
plot(t,RV2);
plot(t,RV3);
plot(t,-RVn);
legend("RV_{1}","RV_{2}","RV_{3}","- RV_{n}");
xlabel("Temps (s)");
ylabel("Tension (V)");
hold off