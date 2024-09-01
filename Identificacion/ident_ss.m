data_out = readtable('OUTPUT.csv');
output = table2array(data_out);

data_in = readtable('INPUT.csv');
input = table2array(data_in);
input = input';

%output_t = output';
%output_corr = output()