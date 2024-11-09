data_out = readtable('Ident_10/OUTPUT.csv');
output = table2array(data_out);

data_in = readtable('Ident_10/INPUT.csv');
input = table2array(data_in);
input = input';

%input = input .* 0.01 * 4095;

%Reemplazar los valores
input(input == 0) = 0;
input(input == 1) = 1000;

%output_t = output';
%output_corr = output()