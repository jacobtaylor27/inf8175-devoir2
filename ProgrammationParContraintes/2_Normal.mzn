% Jacob Taylor (2117518)
% Laurie Bédard-Côté (2086165)

include "globals.mzn";

% États possibles pour une case du terrain
% V = Vide
% B = Tour bleue
% R = Tour rouge
enum etat = {V, B, R};

% Dimension du terrain (carré)
int: n;
set of int: dim = 1..n;

% État du terrain
array[dim, dim] of var etat: terrain;

% Variable objectif à maximiser
var int: total_commun;

%-----------------------------------------------------------------------------%
% Partie étudiant
%-----------------------------------------------------------------------------%

% le nombre de tours de chaque couleur sur un terrain doit etre egal
var int: countR = count([terrain[i, j] | i, j in dim], R);
var int: countB = count([terrain[i, j] | i, j in dim], B);

constraint countR = countB;

% les tours de couleurs diff ne doivent pas interferer leurs signaux sur leur ligne
function var bool: hasClearLineOfSight(array[int] of var etat: line, etat: color) =
    forall(i in index_set(line)) (
        line[i] = color \/ line[i] = V
    );
    
% les tours de courleurs diff ne doivent pas interferer leurs signaux sur leurs diagonales
function var bool: hasClearDiagOfSight(int: x, int: y, etat: color) =
    forall(i, j in 1..n where abs(i - x) = abs(j - y) /\ i != x /\ j != y)(
        terrain[i, j] = color \/ terrain[i, j] = V
    );

% vérifie les lignes, colonnes et diagonales pour éviter les interferences
constraint forall(i, j in dim) (
  if terrain[i,j] = R then
    hasClearLineOfSight(row(terrain,i), R) /\
    hasClearLineOfSight(col(terrain,j), R) /\
    hasClearDiagOfSight(i,j, R)
  else
    true
  endif
);

% Contrainte de symétrie : placement des tours "R" uniquement dans la moitié supérieure du terrain
constraint forall(i, j in dim where i > j) (
    terrain[i, j] != R
);

% Contrainte de symétrie : placement des tours "B" uniquement dans l'autre moitié supérieure du terrain
constraint forall(i, j in dim where i < j) (
    terrain[i, j] != B
);


constraint total_commun = countR;
solve maximize total_commun;
%-----------------------------------------------------------------------------%
output 
["Terrain :\n"]
++
[if j = 1 then "["  endif ++ if fix(terrain[i,j]) == V then "_" else show(terrain[i,j]) endif ++ if j = n then "]\n" else "," endif | i in 1..n, j in 1..n]
++
["\n"]
++
["Total de tours commun: " ++ show(total_commun)]
