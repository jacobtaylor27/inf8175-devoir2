% Jacob Taylor (2117518)
% Laurie Bédard-Côté (2086165)

include "globals.mzn";

% Nombre de fréquences à placer
int: m;
% Placements possibles selon le nombre de fréquences
int: n = m*m;

% Définition de la plage de fréquences 
array[1..m] of var 0..n: frequences;

% Calcul des distances entre les différentes fréquences
array[1..(m*(m-1)) div 2] of var 0..n: distances =
    [ frequences[j] - frequences[i] | i in 1..m, j in i+1..m];

% Variable objectif à minimiser
var int: longueur;

%-----------------------------------------------------------------------------%
% Partie étudiant
%-----------------------------------------------------------------------------%
% chaque freq est un entier en ordre croissant sur la plage
constraint strictly_increasing(frequences);

% premiere freq commence a 0
constraint frequences[1] = 0;

% derniere valeur indique la longueur totale de la plage

% freq rependues a des distance differentes sur la plage : chaque chiffre dans distances est diff
constraint all_different(distances);

longueur = frequences[m];

solve minimize longueur;
%-----------------------------------------------------------------------------%

output 
  [show(frequences)]
  ++
  ["\nLongueur de la plage = ", show(longueur)];
