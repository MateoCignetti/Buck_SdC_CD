clear all:
data_out = readtable('../Identificacion/data_20240830_020530.csv');
output = table2array(data_out);

t = (0:length(output)-1) * 200e-6;

plot(t,output);

%output_t = output';
%output_corr = output()