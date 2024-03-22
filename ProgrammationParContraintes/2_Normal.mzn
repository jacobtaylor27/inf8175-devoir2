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

% les tours de couleurs diff ne doivent pas interferer leurs signaux (h, v, d)
function var bool: hasClearLineOfSight(array[int] of etat: line, etat: color) =
    forall(i in index_set(line)) (
        line[i] == color \/ line[i] == V
    );

constraint forall(i, j in dim) (
  if terrain[i, j] != V then
    let {
      array[int] of etat: row = [terrain[i, k] | k in dim];
      array[int] of etat: col = [terrain[k, j] | k in dim];
      array[int] of etat: diag1 = [terrain[k, k+i-j] | k in 1+i-max(i,j)..dim];
      array[int] of etat: diag2 = [terrain[k, j+i-k] | k in max(i,j)-i+1..min(dim, i+j-1)];
    } in
    hasClearLineOfSight(row, terrain[i, j]) /\
    hasClearLineOfSight(col, terrain[i, j]) /\
    hasClearLineOfSight(diag1, terrain[i, j]) /\
    hasClearLineOfSight(diag2, terrain[i, j])
  else
    true
  endif
);

% symmetry (speed)

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
