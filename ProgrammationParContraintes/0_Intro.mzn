include "globals.mzn";

var 1000..9999: nombre;

array[1..4] of var 1..9: chiffres;

constraint chiffres[1] != 0;
constraint chiffres[2] != 0;
constraint chiffres[3] != 0;
constraint chiffres[4] != 0;

constraint alldifferent(chiffres);

constraint nombre mod 2 = 0;

constraint 

nombre = chiffres[1] * 1000 + chiffres[2] * 100 + chiffres[3] * 10 + chiffres[4];

solve satisfy;

output [
  show(chiffres),
  show(nombre)
];

