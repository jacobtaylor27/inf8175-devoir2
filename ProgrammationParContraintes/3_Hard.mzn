% Jacob Taylor (2117518)
% Laurie Bédard-Côté (2086165)

include "globals.mzn";

% Énumérations des équipes participant au tournoi
enum equipes = {CO, TA, HO, DA, NA, MU, AL, UT};

% Lieux de rencontres des equipes (0 = pas de rencontre).
% Tout lieu i, tel que 0 < i <= card(equipes), 
% est designé comme le domicile de l'équipe associée par la relation to_enum(equipe, i)
set of int: terrains = 0..card(equipes);

% Étendue du tournoi (en jours)
int: d = 16;
set of int: jours = 1..d;

% Calendrier de placement des matchs
array[equipes, jours] of var terrains: calendrier;

% Nombres de rencontres entre deux equipes
array[equipes, equipes] of var 0..2: rencontres;

% Tuples des dates de match entre deux équipes
array[equipes, equipes] of var tuple (0..d, 0..d): dates;

% Variable objectif à maximiser
var int: objectif_match_espaces;

%-----------------------------------------------------------------------------%
% Partie étudiant
%-----------------------------------------------------------------------------%

%1. Pour effectuer une rencontre, deux équipe doivent être présente sur le même terrain 
%6. les equipes doivent jouer 2 fois l'une contre l'autre
%8. Les deux rencontres de deux équipes doivent avoir lieu sur des terrains différents.
constraint forall(e1, e2 in equipes where e1 < e2)(
    calendrier[e1, dates[e1, e2].1] = e1 /\ calendrier[e2, dates[e1, e2].2] = e2 /\
    calendrier[e1, dates[e2, e1].2] = e2 /\ calendrier[e2, dates[e2, e1].1] = e1 /\

    sum(j in jours) ((calendrier[e1, j ] = e2) \/ (calendrier[e2, j] = e1)) = 2 /\
    
    calendrier[e1, dates[e1, e2].1] != calendrier[e1, dates[e1, e2].2] 
);


%2. Un match doit avoir lieu sur le terrain désigné comme le domicile de l’une des deux équipes.
constraint forall(j in jours)(
    forall(e1, e2 in equipes where e1 < e2)(
        (calendrier[e1, j] = e2) -> (calendrier[e2, j] = e2)
    )
);

%3. Un terrain ne peut accueillir plus de deux équipes.
constraint forall(j in jours, t in terrains where t != 0)(
    count([calendrier[e, j] | e in equipes], t) <= 2
);


%5. Une équipe ne doit pas jouer trop souvent à domicile, elle n’est donc pas autorisée à jouer plus de 7 jours sur son propre terrain.
constraint forall(e in equipes)(
    sum(j in jours)(calendrier[e, j] = e) <= 7
);



%7. Les deux rencontres de deux équipes doivent être espacées de 3 jours au minimum.
constraint forall(e1, e2 in equipes where e1 < e2)(
    let {
        var jours: jour1 = dates[e1, e2].1,
        var jours: jour2 = dates[e1, e2].2
    } in (
        jour1 != 0 /\ jour2 != 0 /\ 
        jour1 < jour2 /\ 
        jour2 - jour1 > 3 /\ 
        calendrier[e1, jour1] = calendrier[e2, jour1]  /\
        calendrier[e1, jour2] = calendrier[e2, jour2] 
    )
);

% le nombre de fois où deux matchs entre les mêmes équipes sont espacéspar aux moins 6 jours
constraint objectif_match_espaces = sum(e1, e2 in equipes where e1 < e2)(
    dates[e1, e2].2 - dates[e1, e2].1 > 6
);

solve maximize objectif_match_espaces;
%-----------------------------------------------------------------------------%

% Affichage du calendrier par equipes et par jours puis du résultat de la fonction objectif.
% Lorsqu'une équipe ne joue pas, l'emplacement de match reste vide sinon elle contient le nom de l'équipie adverse.
% Si une équipe ne joue pas à domicile, un "@" précède le nom de l'équipe adverse.
output 
["Calendrier:\n"]
++
[if j = 1 then "   " else "" endif ++ if j < 10 then "  j" ++ show(j) else " j" ++ show(j) endif ++ if j = d then "\n" else "" endif | j in jours ]
++
[if j = 1 then show(e1) ++ ":" else "" endif ++ if fix(calendrier[e1,j]) = 0 then "    " else if fix(calendrier[e1,j]) = e1 then "  " ++ concat([if e1 != e2 /\ fix(calendrier[e2,j]) = e1 then show(e2) else "" endif | e2 in equipes])  else " @" ++ show(to_enum(equipes, calendrier[e1,j])) endif endif ++ if j = d then "\n" else "" endif | e1 in equipes, j in jours ]
++
["\nMatchs retour espacés de 6 jours ou plus = " ++ show(objectif_match_espaces) ++ "\n"]
;
